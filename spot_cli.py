import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _wait_for_subscribers(node: Node, pub, wait_s: float) -> int:
    deadline = time.monotonic() + max(0.0, float(wait_s))
    count = int(pub.get_subscription_count())
    while count < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)
        count = int(pub.get_subscription_count())
    return count


def _publish(
    topic: str,
    payload: dict,
    repeat: int = 3,
    sleep_s: float = 0.05,
    wait_s: float = 1.0,
) -> None:
    rclpy.init()
    node = Node("spot_cli")
    pub = node.create_publisher(String, topic, 10)

    msg = String()
    msg.data = json.dumps(payload, separators=(",", ":"))

    try:
        sub_count = _wait_for_subscribers(node, pub, wait_s)
        if sub_count < 1 and wait_s > 0:
            print(f"warning: no subscribers discovered on {topic} after {wait_s:.1f}s; sending anyway")

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
    parser.add_argument("--wait", type=float, default=1.0)

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

    _publish(args.topic, payload, repeat=args.repeat, sleep_s=args.sleep, wait_s=args.wait)


if __name__ == "__main__":
    main()
