import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _publish(topic: str, payload: dict, repeat: int = 3, sleep_s: float = 0.05) -> None:
    rclpy.init()
    node = Node("spot_cli")
    pub = node.create_publisher(String, topic, 10)

    msg = String()
    msg.data = json.dumps(payload, separators=(",", ":"))

    try:
        for _ in range(max(1, int(repeat))):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, float(sleep_s)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    default_topic = (
        os.environ.get("CMD_TOPIC", "/spot/cmd/servo").strip() or "/spot/cmd/servo"
    )

    parser = argparse.ArgumentParser(
        description="Spot servo command helper (publishes JSON to std_msgs/String)."
    )
    parser.add_argument("--topic", default=default_topic)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--sleep", type=float, default=0.05)

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("estop")
    sub.add_parser("clear-estop")
    sub.add_parser("home")

    p_set = sub.add_parser("set", help="Set joints as normalized [-1..1] values.")
    p_set.add_argument("pairs", nargs="+", help="Pairs like rf_hip=0.1 lf_hip=-0.1")

    p_set_us = sub.add_parser(
        "set-us", help="Set joints as absolute pulse width in microseconds."
    )
    p_set_us.add_argument("pairs", nargs="+", help="Pairs like rf_hip=1500 lf_hip=1520")

    args = parser.parse_args()

    if args.command == "arm":
        payload = {"cmd": "arm", "value": True}
    elif args.command == "disarm":
        payload = {"cmd": "arm", "value": False}
    elif args.command == "estop":
        payload = {"cmd": "estop", "value": True}
    elif args.command == "clear-estop":
        payload = {"cmd": "estop", "value": False}
    elif args.command == "home":
        payload = {"cmd": "home"}
    elif args.command in ("set", "set-us"):
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

        payload = {
            "cmd": "set",
            "mode": "us" if args.command == "set-us" else "norm",
            "targets": targets,
        }
    else:
        raise SystemExit(f"unknown command: {args.command}")

    _publish(args.topic, payload, repeat=args.repeat, sleep_s=args.sleep)


if __name__ == "__main__":
    main()
