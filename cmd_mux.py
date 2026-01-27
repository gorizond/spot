from __future__ import annotations

import json
import os
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

AUTO_TOPIC = os.environ.get("AUTO_TOPIC", "/spot/cmd/servo_auto").strip() or "/spot/cmd/servo_auto"
MANUAL_TOPIC = (
    os.environ.get("MANUAL_TOPIC", "/spot/cmd/servo_manual").strip()
    or "/spot/cmd/servo_manual"
)
OUT_TOPIC = os.environ.get("OUT_TOPIC", "/spot/cmd/servo").strip() or "/spot/cmd/servo"
MODE_TOPIC = os.environ.get("MODE_TOPIC", "/spot/ctrl/mode").strip() or "/spot/ctrl/mode"
STATUS_TOPIC = (
    os.environ.get("STATUS_TOPIC", "/spot/state/mux").strip() or "/spot/state/mux"
)
DEFAULT_MODE = os.environ.get("DEFAULT_MODE", "auto").strip().lower() or "auto"
MANUAL_TIMEOUT_S = float(os.environ.get("MANUAL_TIMEOUT_S", "0"))
STATUS_PUBLISH_S = float(os.environ.get("STATUS_PUBLISH_S", "1.0"))

_stop = False


def _handle_stop(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)


def _normalize_mode(raw: str) -> str | None:
    val = str(raw or "").strip().lower()
    if val in ("auto", "manual"):
        return val
    return None


class CmdMux(Node):
    def __init__(self) -> None:
        super().__init__("spot_cmd_mux")

        self._mode = _normalize_mode(DEFAULT_MODE) or "auto"
        self._last_auto: str | None = None
        self._last_manual: str | None = None
        self._last_manual_ts = 0.0
        self._last_status = 0.0

        self._pub = self.create_publisher(String, OUT_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self.create_subscription(String, AUTO_TOPIC, self._on_auto, 10)
        self.create_subscription(String, MANUAL_TOPIC, self._on_manual, 10)
        self.create_subscription(String, MODE_TOPIC, self._on_mode, 10)

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"starting; mode={self._mode} auto={AUTO_TOPIC} manual={MANUAL_TOPIC} out={OUT_TOPIC}"
        )

    def _publish(self, payload: str) -> None:
        if not payload:
            return
        msg = String()
        msg.data = payload
        self._pub.publish(msg)

    def _on_auto(self, msg: String) -> None:
        payload = (msg.data or "").strip()
        if not payload:
            return
        self._last_auto = payload
        if self._mode == "auto":
            self._publish(payload)

    def _on_manual(self, msg: String) -> None:
        payload = (msg.data or "").strip()
        if not payload:
            return
        self._last_manual = payload
        self._last_manual_ts = time.monotonic()
        if self._mode == "manual":
            self._publish(payload)

    def _on_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        mode = _normalize_mode(raw)
        if not mode:
            return
        if mode == self._mode:
            return
        self._mode = mode
        self.get_logger().info(f"mode={self._mode}")
        if self._mode == "auto" and self._last_auto:
            self._publish(self._last_auto)
        if self._mode == "manual" and self._last_manual:
            self._publish(self._last_manual)

    def _publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status < max(0.1, STATUS_PUBLISH_S):
            return
        self._last_status = now

        payload = {
            "mode": self._mode,
            "auto_topic": AUTO_TOPIC,
            "manual_topic": MANUAL_TOPIC,
            "out_topic": OUT_TOPIC,
            "manual_timeout_s": MANUAL_TIMEOUT_S,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _tick(self) -> None:
        if MANUAL_TIMEOUT_S > 0 and self._mode == "manual":
            if self._last_manual_ts > 0 and (time.monotonic() - self._last_manual_ts) > MANUAL_TIMEOUT_S:
                self._mode = "auto"
                if self._last_auto:
                    self._publish(self._last_auto)
                self.get_logger().info("manual timeout; switching to auto")

        self._publish_status()


def main() -> None:
    rclpy.init()
    node = CmdMux()
    try:
        while rclpy.ok() and not _stop:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
