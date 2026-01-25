from __future__ import annotations

import argparse
import json
import os
import re
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_STAND_HIP = float(os.environ.get("STAND_HIP", "0.02"))
DEFAULT_STAND_UPPER = float(os.environ.get("STAND_UPPER", "0.05"))
DEFAULT_STAND_LOWER = float(os.environ.get("STAND_LOWER", "0.05"))
DEFAULT_STAND_HIP_REAR_OFFSET = float(os.environ.get("STAND_HIP_REAR_OFFSET", "0.0"))
DEFAULT_LIFT_DELTA = float(os.environ.get("LIFT_DELTA", "0.10"))
DEFAULT_WALK_ORDER = os.environ.get("WALK_ORDER", "lh,rf,lf,rh")

_stop = False


def _handle_stop(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)


def _wait_for_subscribers(node: Node, pub, wait_s: float) -> int:
    deadline = time.monotonic() + max(0.0, float(wait_s))
    count = int(pub.get_subscription_count())
    while count < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)
        count = int(pub.get_subscription_count())
    return count


def _clamp_norm(val: float) -> float:
    try:
        val = float(val)
    except Exception:
        return 0.0

    if val < -1.0:
        return -1.0
    if val > 1.0:
        return 1.0
    return val


def _parse_walk_order(raw: str) -> list[str]:
    raw = str(raw or "").strip()
    parts = [p for p in re.split(r"[,\s]+", raw) if p]

    allowed = {"rf", "lf", "rh", "lh"}
    order = [p for p in parts if p in allowed]

    return order or ["lh", "rf", "lf", "rh"]


def _stand_targets(hip: float, upper: float, lower: float, rear_hip_offset: float) -> dict:
    targets = {}
    for leg in ("rf", "lf", "rh", "lh"):
        hip_val = float(hip)
        if leg in ("rh", "lh"):
            hip_val = float(hip) + float(rear_hip_offset)
        targets[f"{leg}_hip"] = _clamp_norm(hip_val)
        targets[f"{leg}_upper"] = _clamp_norm(upper)
        targets[f"{leg}_lower"] = _clamp_norm(lower)
    return targets


def _set_payload(targets: dict, mode: str = "norm") -> dict:
    return {"cmd": "set", "mode": mode, "targets": targets}


class SpotPublisher:
    def __init__(self, topic: str, wait_s: float):
        rclpy.init()
        self._node = Node("spot_cli")
        self._pub = self._node.create_publisher(String, topic, 10)

        sub_count = _wait_for_subscribers(self._node, self._pub, wait_s)
        if sub_count < 1 and wait_s > 0:
            print(
                f"warning: no subscribers discovered on {topic} after {wait_s:.1f}s; sending anyway"
            )

    def publish(self, payload: dict, repeat: int, sleep_s: float) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))

        for _ in range(max(1, int(repeat))):
            if _stop:
                return
            self._pub.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.05)
            time.sleep(max(0.0, float(sleep_s)))

    def sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, float(seconds))
        while not _stop and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            time.sleep(0.02)

    def close(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()


def _run_step_sequence(
    pub: SpotPublisher,
    *,
    stand_hip: float,
    stand_upper: float,
    stand_lower: float,
    stand_rear_hip_offset: float,
    leg: str,
    lift_delta: float,
    hip_swing: float,
    lift_hold_s: float,
    swing_hold_s: float,
    down_hold_s: float,
    return_hold_s: float,
    repeat: int,
    sleep_s: float,
) -> None:
    base_targets = _stand_targets(stand_hip, stand_upper, stand_lower, stand_rear_hip_offset)
    pub.publish(_set_payload(base_targets), repeat=repeat, sleep_s=sleep_s)
    pub.sleep(max(0.0, float(return_hold_s)))

    leg = str(leg or "").strip().lower()
    if leg not in ("rf", "lf", "rh", "lh"):
        raise SystemExit("step: leg must be one of rf/lf/rh/lh")

    lifted_targets = dict(base_targets)
    lifted_targets[f"{leg}_lower"] = _clamp_norm(float(stand_lower) - float(lift_delta))
    pub.publish(_set_payload(lifted_targets), repeat=repeat, sleep_s=sleep_s)
    pub.sleep(max(0.0, float(lift_hold_s)))

    if abs(float(hip_swing)) > 1e-6:
        swung_targets = dict(lifted_targets)
        swung_targets[f"{leg}_hip"] = _clamp_norm(float(stand_hip) + float(hip_swing))
        pub.publish(_set_payload(swung_targets), repeat=repeat, sleep_s=sleep_s)
        pub.sleep(max(0.0, float(swing_hold_s)))

        down_targets = dict(swung_targets)
        down_targets[f"{leg}_lower"] = _clamp_norm(stand_lower)
        pub.publish(_set_payload(down_targets), repeat=repeat, sleep_s=sleep_s)
        pub.sleep(max(0.0, float(down_hold_s)))

    down_targets = dict(base_targets)
    down_targets[f"{leg}_lower"] = _clamp_norm(stand_lower)
    pub.publish(_set_payload(down_targets), repeat=repeat, sleep_s=sleep_s)
    pub.sleep(max(0.0, float(down_hold_s)))

    pub.publish(_set_payload(base_targets), repeat=repeat, sleep_s=sleep_s)
    pub.sleep(max(0.0, float(return_hold_s)))


def _run_walk_sequence(
    pub: SpotPublisher,
    *,
    steps: int,
    order: list[str],
    stand_hip: float,
    stand_upper: float,
    stand_lower: float,
    stand_rear_hip_offset: float,
    lift_delta: float,
    hip_swing: float,
    lift_hold_s: float,
    swing_hold_s: float,
    down_hold_s: float,
    between_legs_s: float,
    repeat: int,
    sleep_s: float,
    return_to_stand: bool,
) -> None:
    base_targets = _stand_targets(stand_hip, stand_upper, stand_lower, stand_rear_hip_offset)
    pub.publish(_set_payload(base_targets), repeat=repeat, sleep_s=sleep_s)

    for _ in range(max(1, int(steps))):
        for leg in order:
            if _stop:
                break

            _run_step_sequence(
                pub,
                stand_hip=stand_hip,
                stand_upper=stand_upper,
                stand_lower=stand_lower,
                stand_rear_hip_offset=stand_rear_hip_offset,
                leg=leg,
                lift_delta=lift_delta,
                hip_swing=hip_swing,
                lift_hold_s=lift_hold_s,
                swing_hold_s=swing_hold_s,
                down_hold_s=down_hold_s,
                return_hold_s=0.0,
                repeat=repeat,
                sleep_s=sleep_s,
            )
            pub.sleep(max(0.0, float(between_legs_s)))

        if _stop:
            break

    if return_to_stand and not _stop:
        pub.publish(_set_payload(base_targets), repeat=repeat, sleep_s=sleep_s)


def main() -> None:
    default_topic = os.environ.get("CMD_TOPIC", "/spot/cmd/servo").strip() or "/spot/cmd/servo"

    parser = argparse.ArgumentParser(
        description="Spot servo command helper (publishes JSON to std_msgs/String)."
    )
    parser.add_argument("--topic", default=default_topic)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--sleep", type=float, default=0.05)
    parser.add_argument("--wait", type=float, default=1.0)

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("estop")
    sub.add_parser("clear-estop")
    sub.add_parser("home")

    p_stand = sub.add_parser(
        "stand", help="Set a conservative stand pose (enables all 12 joints)."
    )
    p_stand.add_argument("--hip", type=float, default=DEFAULT_STAND_HIP)
    p_stand.add_argument("--upper", type=float, default=DEFAULT_STAND_UPPER)
    p_stand.add_argument("--lower", type=float, default=DEFAULT_STAND_LOWER)
    p_stand.add_argument(
        "--rear-hip-offset",
        type=float,
        default=DEFAULT_STAND_HIP_REAR_OFFSET,
        help="Extra hip offset for rear legs (rh/lh) only.",
    )

    p_step = sub.add_parser("step", help="One safe micro-step (lift/return one leg).")
    p_step.add_argument("leg", help="Leg prefix: rf/lf/rh/lh")
    p_step.add_argument("--stand-hip", type=float, default=DEFAULT_STAND_HIP)
    p_step.add_argument("--stand-upper", type=float, default=DEFAULT_STAND_UPPER)
    p_step.add_argument("--stand-lower", type=float, default=DEFAULT_STAND_LOWER)
    p_step.add_argument(
        "--stand-rear-hip-offset",
        type=float,
        default=DEFAULT_STAND_HIP_REAR_OFFSET,
        help="Extra hip offset for rear legs (rh/lh) only.",
    )
    p_step.add_argument("--lift", type=float, default=DEFAULT_LIFT_DELTA)
    p_step.add_argument(
        "--hip-swing",
        type=float,
        default=0.0,
        help="Optional hip offset applied while leg is lifted (experimental).",
    )
    p_step.add_argument("--lift-hold", type=float, default=0.25)
    p_step.add_argument("--swing-hold", type=float, default=0.25)
    p_step.add_argument("--down-hold", type=float, default=0.25)
    p_step.add_argument("--return-hold", type=float, default=0.10)

    p_walk = sub.add_parser(
        "walk",
        help="Repeat micro-steps (in-place crawl). Starts from stand pose.",
    )
    p_walk.add_argument("--steps", type=int, default=1)
    p_walk.add_argument("--order", default=DEFAULT_WALK_ORDER)
    p_walk.add_argument("--stand-hip", type=float, default=DEFAULT_STAND_HIP)
    p_walk.add_argument("--stand-upper", type=float, default=DEFAULT_STAND_UPPER)
    p_walk.add_argument("--stand-lower", type=float, default=DEFAULT_STAND_LOWER)
    p_walk.add_argument(
        "--stand-rear-hip-offset",
        type=float,
        default=DEFAULT_STAND_HIP_REAR_OFFSET,
        help="Extra hip offset for rear legs (rh/lh) only.",
    )
    p_walk.add_argument("--lift", type=float, default=DEFAULT_LIFT_DELTA)
    p_walk.add_argument(
        "--hip-swing",
        type=float,
        default=0.0,
        help="Optional hip offset applied while a leg is lifted (experimental).",
    )
    p_walk.add_argument("--lift-hold", type=float, default=0.25)
    p_walk.add_argument("--swing-hold", type=float, default=0.25)
    p_walk.add_argument("--down-hold", type=float, default=0.25)
    p_walk.add_argument("--between-legs", type=float, default=0.05)
    p_walk.add_argument(
        "--no-return-to-stand",
        action="store_true",
        help="Do not send stand pose at the end.",
    )

    p_set = sub.add_parser("set", help="Set joints as normalized [-1..1] values.")
    p_set.add_argument("pairs", nargs="+", help="Pairs like rf_hip=0.1 lf_hip=-0.1")

    p_set_us = sub.add_parser(
        "set-us", help="Set joints as absolute pulse width in microseconds."
    )
    p_set_us.add_argument("pairs", nargs="+", help="Pairs like rf_hip=1500 lf_hip=1520")

    args = parser.parse_args()

    pub = SpotPublisher(args.topic, wait_s=args.wait)
    try:
        if args.command == "arm":
            pub.publish({"cmd": "arm", "value": True}, repeat=args.repeat, sleep_s=args.sleep)
            return
        if args.command == "disarm":
            pub.publish({"cmd": "arm", "value": False}, repeat=args.repeat, sleep_s=args.sleep)
            return
        if args.command == "estop":
            pub.publish({"cmd": "estop", "value": True}, repeat=args.repeat, sleep_s=args.sleep)
            return
        if args.command == "clear-estop":
            pub.publish({"cmd": "estop", "value": False}, repeat=args.repeat, sleep_s=args.sleep)
            return
        if args.command == "home":
            pub.publish({"cmd": "home"}, repeat=args.repeat, sleep_s=args.sleep)
            return

        if args.command == "stand":
            targets = _stand_targets(
                args.hip,
                args.upper,
                args.lower,
                args.rear_hip_offset,
            )
            pub.publish(_set_payload(targets), repeat=args.repeat, sleep_s=args.sleep)
            return

        if args.command == "step":
            _run_step_sequence(
                pub,
                stand_hip=args.stand_hip,
                stand_upper=args.stand_upper,
                stand_lower=args.stand_lower,
                stand_rear_hip_offset=args.stand_rear_hip_offset,
                leg=args.leg,
                lift_delta=args.lift,
                hip_swing=args.hip_swing,
                lift_hold_s=args.lift_hold,
                swing_hold_s=args.swing_hold,
                down_hold_s=args.down_hold,
                return_hold_s=args.return_hold,
                repeat=args.repeat,
                sleep_s=args.sleep,
            )
            return

        if args.command == "walk":
            order = _parse_walk_order(args.order)
            _run_walk_sequence(
                pub,
                steps=args.steps,
                order=order,
                stand_hip=args.stand_hip,
                stand_upper=args.stand_upper,
                stand_lower=args.stand_lower,
                stand_rear_hip_offset=args.stand_rear_hip_offset,
                lift_delta=args.lift,
                hip_swing=args.hip_swing,
                lift_hold_s=args.lift_hold,
                swing_hold_s=args.swing_hold,
                down_hold_s=args.down_hold,
                between_legs_s=args.between_legs,
                repeat=args.repeat,
                sleep_s=args.sleep,
                return_to_stand=not args.no_return_to_stand,
            )
            return

        if args.command in ("set", "set-us"):
            targets = {}
            for pair in args.pairs:
                if "=" not in pair:
                    continue
                joint, raw = pair.split("=", 1)
                joint = joint.strip()
                raw = raw.strip()
                if not joint:
                    continue
                try:
                    val = float(raw) if args.command == "set" else int(raw, 0)
                except Exception:
                    continue
                targets[joint] = val

            payload = _set_payload(targets, mode="us" if args.command == "set-us" else "norm")
            pub.publish(payload, repeat=args.repeat, sleep_s=args.sleep)
            return

        raise SystemExit(f"unknown command: {args.command}")
    finally:
        pub.close()


if __name__ == "__main__":
    main()
