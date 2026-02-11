from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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
STAND_HIP_REAR_OFFSET = _parse_float(os.environ.get("STAND_HIP_REAR_OFFSET", "0.0"), 0.0)
STAND_REAR_UPPER_OFFSET = _parse_float(os.environ.get("STAND_REAR_UPPER_OFFSET", "0.0"), 0.0)
STAND_REAR_LOWER_OFFSET = _parse_float(os.environ.get("STAND_REAR_LOWER_OFFSET", "0.0"), 0.0)

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

_LEG_BY_WHERE_SIDE = {
    ("front", "left"): "lf",
    ("front", "right"): "rf",
    ("rear", "left"): "lh",
    ("rear", "right"): "rh",
}

_PART_TO_KIND_SUFFIX = {
    "shoulder": ("hip", "hip"),
    "leg": ("upper", "upper"),
    "foot": ("lower", "lower"),
}


def _map_champ_joint_to_spot(name: str) -> tuple[str, str] | None:
    parts = str(name or "").strip().split("_")
    if len(parts) != 3:
        return None

    where, side, part = parts
    leg = _LEG_BY_WHERE_SIDE.get((where, side))
    if not leg:
        return None

    kind_suffix = _PART_TO_KIND_SUFFIX.get(part)
    if not kind_suffix:
        return None

    kind, suffix = kind_suffix
    return f"{leg}_{suffix}", kind


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

        self._latest_joint_state: JointState | None = None
        self._latest_joint_state_seq = 0
        self._processed_joint_state_seq = 0
        self._force_publish = False

        self._zero_by_joint: dict[str, float] = {}

        self._gain = float(_clamp(gain, 0.0, 2.0))
        self._publish_period_s = 1.0 / max(1.0, float(publish_hz))
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
        self._stand_rear_hip_offset = float(_clamp(STAND_HIP_REAR_OFFSET, -1.0, 1.0))
        self._stand_rear_upper_offset = float(_clamp(STAND_REAR_UPPER_OFFSET, -1.0, 1.0))
        self._stand_rear_lower_offset = float(_clamp(STAND_REAR_LOWER_OFFSET, -1.0, 1.0))

        qos_joint_states = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(String, status_topic, self._on_status, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, qos_joint_states
        )
        self.create_timer(self._publish_period_s, self._tick)

        self.get_logger().info(
            "starting; joint_states=%s status=%s cmd=%s publish_hz=%.1f gain=%.2f stand=(%.3f/%.3f/%.3f) rear_hip_offset=%.3f rear_upper_offset=%.3f rear_lower_offset=%.3f"
            % (
                joint_states_topic,
                status_topic,
                cmd_topic,
                1.0 / self._publish_period_s,
                self._gain,
                self._stand["hip"],
                self._stand["upper"],
                self._stand["lower"],
                self._stand_rear_hip_offset,
                self._stand_rear_upper_offset,
                self._stand_rear_lower_offset,
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

        armed = bool(obj.get("armed", False))
        estop = bool(obj.get("estop", False))

        if armed != self._armed or estop != self._estop:
            self._force_publish = True

        self._armed = armed
        self._estop = estop

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        self._latest_joint_state_seq += 1

    def _tick(self) -> None:
        if self._estop:
            return

        if self._require_armed and not self._armed:
            return

        msg = self._latest_joint_state
        if msg is None:
            return

        if (
            self._latest_joint_state_seq == self._processed_joint_state_seq
            and not self._force_publish
        ):
            return

        names = msg.name
        positions = msg.position
        if not names or not positions or len(names) != len(positions):
            return

        if not self._zero_by_joint:
            self._zero_by_joint = {
                str(name): float(pos)
                for name, pos in zip(names, positions)
                if name
            }
            self.get_logger().info(
                "captured CHAMP zero angles (stand reference) from first joint_states message"
            )

        zero_by_joint = self._zero_by_joint
        ranges = self._ranges
        stand = self._stand
        rear_hip_offset = self._stand_rear_hip_offset
        rear_upper_offset = self._stand_rear_upper_offset
        rear_lower_offset = self._stand_rear_lower_offset
        gain = self._gain

        targets: dict[str, float] = {}
        for name, pos in zip(names, positions):
            mapped = _map_champ_joint_to_spot(name)
            if not mapped:
                continue

            out_name, kind = mapped
            zero = float(zero_by_joint.get(name, 0.0))
            delta = float(pos) - zero

            scaled = delta / ranges[kind]
            scaled = float(_clamp(scaled, -1.0, 1.0))

            stand_base = stand[kind]
            if out_name in ("rh_hip", "lh_hip"):
                if kind == "hip":
                    stand_base = stand_base + rear_hip_offset
                elif kind == "upper":
                    stand_base = stand_base + rear_upper_offset
                elif kind == "lower":
                    stand_base = stand_base + rear_lower_offset
            out = stand_base + scaled * gain
            out = float(_clamp(out, -1.0, 1.0))
            targets[out_name] = out

        self._processed_joint_state_seq = self._latest_joint_state_seq
        self._force_publish = False

        if not targets:
            return

        payload = {"cmd": "set", "mode": "norm", "targets": targets}
        out_msg = String()
        out_msg.data = json.dumps(payload, separators=(",", ":"))
        self._cmd_pub.publish(out_msg)

        now = time.monotonic()
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().info(
                "published %d joint targets to %s (armed=%s)"
                % (
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
    parser.add_argument("--cmd-topic", default="/spot/cmd/servo_auto")
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
