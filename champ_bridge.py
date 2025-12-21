from __future__ import annotations

import argparse
import json
import os
import re
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _parse_float(raw: str, default: float) -> float:
    try:
        return float(str(raw).strip())
    except Exception:
        return default


STAND_HIP = _parse_float(os.environ.get("STAND_HIP", "0.02"), 0.02)
STAND_UPPER = _parse_float(os.environ.get("STAND_UPPER", "0.05"), 0.05)
STAND_LOWER = _parse_float(os.environ.get("STAND_LOWER", "0.05"), 0.05)

GAIN = _parse_float(os.environ.get("CHAMP_GAIN", "0.25"), 0.25)
PUBLISH_HZ = _parse_float(os.environ.get("CHAMP_BRIDGE_HZ", "50"), 50.0)

HIP_RANGE_RAD = _parse_float(os.environ.get("CHAMP_HIP_RANGE_RAD", "0.55"), 0.55)
UPPER_RANGE_RAD = _parse_float(os.environ.get("CHAMP_UPPER_RANGE_RAD", "1.0"), 1.0)
LOWER_RANGE_RAD = _parse_float(os.environ.get("CHAMP_LOWER_RANGE_RAD", "1.0"), 1.0)

REQUIRE_ARMED = str(os.environ.get("CHAMP_REQUIRE_ARMED", "1")).strip().lower() not in (
    "0",
    "false",
    "no",
    "n",
    "off",
)


def _map_champ_joint_to_spot(name: str) -> tuple[str, str] | None:
    name = str(name or "").strip()
    m = re.match(r"^(front|rear)_(left|right)_(shoulder|leg|foot)$", name)
    if not m:
        return None

    where, side, part = m.groups()
    if where == "front" and side == "left":
        leg = "lf"
    elif where == "front" and side == "right":
        leg = "rf"
    elif where == "rear" and side == "left":
        leg = "lh"
    else:
        leg = "rh"

    if part == "shoulder":
        kind = "hip"
        out = f"{leg}_hip"
    elif part == "leg":
        kind = "upper"
        out = f"{leg}_upper"
    else:
        kind = "lower"
        out = f"{leg}_lower"

    return out, kind


class ChampBridge(Node):
    def __init__(
        self,
        *,
        joint_states_topic: str,
        status_topic: str,
        cmd_topic: str,
        publish_hz: float,
        gain: float,
        hip_range_rad: float,
        upper_range_rad: float,
        lower_range_rad: float,
        require_armed: bool,
    ):
        super().__init__("spot_champ_bridge")

        self._cmd_topic = cmd_topic
        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)

        self._armed = False
        self._estop = False
        self._require_armed = bool(require_armed)

        self._zero_by_joint: dict[str, float] = {}

        self._gain = float(_clamp(gain, 0.0, 2.0))
        self._publish_period_s = 1.0 / max(1.0, float(publish_hz))
        self._last_pub = 0.0
        self._last_log = 0.0

        self._ranges = {
            "hip": max(1e-6, float(hip_range_rad)),
            "upper": max(1e-6, float(upper_range_rad)),
            "lower": max(1e-6, float(lower_range_rad)),
        }
        self._stand = {
            "hip": float(_clamp(STAND_HIP, -1.0, 1.0)),
            "upper": float(_clamp(STAND_UPPER, -1.0, 1.0)),
            "lower": float(_clamp(STAND_LOWER, -1.0, 1.0)),
        }

        self.create_subscription(String, status_topic, self._on_status, 10)
        self.create_subscription(JointState, joint_states_topic, self._on_joint_states, 10)

        self.get_logger().info(
            "starting; joint_states=%s status=%s cmd=%s publish_hz=%.1f gain=%.2f stand=(%.3f/%.3f/%.3f)" % (
                joint_states_topic,
                status_topic,
                cmd_topic,
                1.0 / self._publish_period_s,
                self._gain,
                self._stand["hip"],
                self._stand["upper"],
                self._stand["lower"],
            )
        )

    def _on_status(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return

        try:
            obj = json.loads(raw)
        except Exception:
            return

        if not isinstance(obj, dict):
            return

        self._armed = bool(obj.get("armed", False))
        self._estop = bool(obj.get("estop", False))

    def _on_joint_states(self, msg: JointState) -> None:
        now = time.monotonic()
        if now - self._last_pub < self._publish_period_s:
            return

        if self._estop:
            return

        if self._require_armed and not self._armed:
            return

        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return

        if not self._zero_by_joint:
            self._zero_by_joint = {
                str(name): float(pos)
                for name, pos in zip(msg.name, msg.position)
                if name
            }
            self.get_logger().info(
                "captured CHAMP zero angles (stand reference) from first joint_states message"
            )

        targets: dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            mapped = _map_champ_joint_to_spot(name)
            if not mapped:
                continue

            out_name, kind = mapped
            zero = float(self._zero_by_joint.get(name, 0.0))
            delta = float(pos) - zero

            scaled = float(_clamp(delta / self._ranges[kind], -1.0, 1.0))
            out = self._stand[kind] + scaled * self._gain
            out = float(_clamp(out, -1.0, 1.0))
            targets[out_name] = out

        if not targets:
            return

        payload = {"cmd": "set", "mode": "norm", "targets": targets}
        out_msg = String()
        out_msg.data = json.dumps(payload, separators=(",", ":"))
        self._cmd_pub.publish(out_msg)

        self._last_pub = now
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().info(
                "published %d joint targets to %s (armed=%s)" % (
                    len(targets),
                    self._cmd_topic,
                    "true" if self._armed else "false",
                )
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bridge CHAMP joint_states -> spot servo_driver (/spot/cmd/servo)"
    )
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--status-topic", default="/spot/state/servo")
    parser.add_argument("--cmd-topic", default="/spot/cmd/servo")
    parser.add_argument("--publish-hz", type=float, default=PUBLISH_HZ)
    parser.add_argument("--gain", type=float, default=GAIN)
    parser.add_argument("--hip-range", type=float, default=HIP_RANGE_RAD)
    parser.add_argument("--upper-range", type=float, default=UPPER_RANGE_RAD)
    parser.add_argument("--lower-range", type=float, default=LOWER_RANGE_RAD)
    parser.add_argument(
        "--require-armed",
        action=argparse.BooleanOptionalAction,
        default=REQUIRE_ARMED,
    )

    args = parser.parse_args()

    rclpy.init()
    node = ChampBridge(
        joint_states_topic=args.joint_states_topic,
        status_topic=args.status_topic,
        cmd_topic=args.cmd_topic,
        publish_hz=args.publish_hz,
        gain=args.gain,
        hip_range_rad=args.hip_range,
        upper_range_rad=args.upper_range,
        lower_range_rad=args.lower_range,
        require_armed=args.require_armed,
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
