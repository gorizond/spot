import json
import os
import re
import signal
import ssl
import time
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

K8S_BASE = os.environ.get("K8S_BASE", "https://kubernetes.default.svc").strip()
SA_TOKEN_PATH = "/var/run/secrets/kubernetes.io/serviceaccount/token"
SA_CA_PATH = "/var/run/secrets/kubernetes.io/serviceaccount/ca.crt"

NODE_NAME = os.environ.get("NODE_NAME", "").strip()
LABEL_PREFIX = os.environ.get("LABEL_PREFIX", "gorizond.io/spot-pca9685-").strip()
TOPIC = os.environ.get("TOPIC", "/spot/config/servo_map").strip() or "/spot/config/servo_map"

WATCH_TIMEOUT_SECONDS = int(os.environ.get("WATCH_TIMEOUT_SECONDS", "30"))
REQUEST_TIMEOUT_SECONDS = int(os.environ.get("REQUEST_TIMEOUT_SECONDS", "35"))
BACKOFF_MAX_SECONDS = int(os.environ.get("BACKOFF_MAX_SECONDS", "30"))

_stop = False


def _handle_stop(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read().strip()


def _parse_int(raw: str, default: int) -> int:
    try:
        return int(str(raw).strip(), 0)
    except Exception:
        return default


def _parse_us_triplet(raw):
    if raw is None:
        return None

    nums = re.findall(r"\d+", str(raw))
    if len(nums) < 3:
        return None

    try:
        return int(nums[0]), int(nums[1]), int(nums[2])
    except Exception:
        return None


def _k8s_request(url: str) -> urllib.request.Request:
    token = _read_text(SA_TOKEN_PATH)
    return urllib.request.Request(
        url,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/json",
        },
    )


def _k8s_ssl_context() -> ssl.SSLContext:
    return ssl.create_default_context(cafile=SA_CA_PATH)


def k8s_get_json(path: str) -> dict:
    url = f"{K8S_BASE}{path}"
    req = _k8s_request(url)
    with urllib.request.urlopen(
        req,
        context=_k8s_ssl_context(),
        timeout=REQUEST_TIMEOUT_SECONDS,
    ) as resp:
        body = resp.read()
    obj = json.loads(body)
    return obj if isinstance(obj, dict) else {}


def k8s_watch_node_events(node_name: str, resource_version: str):
    params = {
        "watch": "true",
        "fieldSelector": f"metadata.name={node_name}",
        "resourceVersion": resource_version,
        "timeoutSeconds": str(WATCH_TIMEOUT_SECONDS),
    }
    url = f"{K8S_BASE}/api/v1/nodes?{urllib.parse.urlencode(params)}"
    req = _k8s_request(url)
    with urllib.request.urlopen(
        req,
        context=_k8s_ssl_context(),
        timeout=REQUEST_TIMEOUT_SECONDS,
    ) as resp:
        for raw_line in resp:
            if _stop:
                return
            if not raw_line:
                return
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except Exception:
                continue
            if isinstance(event, dict):
                yield event


def parse_servo_map(labels: dict) -> dict:
    bus = _parse_int(labels.get(f"{LABEL_PREFIX}i2c-bus", "1"), 1)
    address = _parse_int(labels.get(f"{LABEL_PREFIX}address", "0x40"), 0x40)

    channels = []
    for ch in range(16):
        joint = str(labels.get(f"{LABEL_PREFIX}ch{ch}-joint", "")).strip()

        defaults = (1000, 1500, 2000)
        raw_triplet = labels.get(f"{LABEL_PREFIX}ch{ch}-us")
        triplet = _parse_us_triplet(raw_triplet)

        if triplet is not None:
            ch_min, ch_center, ch_max = triplet
        else:
            ch_min = _parse_int(labels.get(f"{LABEL_PREFIX}ch{ch}-min-us", str(defaults[0])), defaults[0])
            ch_center = _parse_int(labels.get(f"{LABEL_PREFIX}ch{ch}-center-us", str(defaults[1])), defaults[1])
            ch_max = _parse_int(labels.get(f"{LABEL_PREFIX}ch{ch}-max-us", str(defaults[2])), defaults[2])

        raw_invert = str(labels.get(f"{LABEL_PREFIX}ch{ch}-invert", "0")).strip().lower()
        ch_invert = raw_invert in ("1", "true", "yes", "y", "on")

        channels.append(
            {
                "ch": ch,
                "joint": joint,
                "min_us": ch_min,
                "center_us": ch_center,
                "max_us": ch_max,
                "invert": ch_invert,
            }
        )

    return {
        "pca9685": {"i2c_bus": bus, "address": address},
        "channels": channels,
    }


def publish_if_changed(node: Node, pub, labels: dict, last_payload):
    servo_map = parse_servo_map(labels)
    payload = json.dumps(servo_map, sort_keys=True, separators=(",", ":"))
    if payload == last_payload:
        return last_payload

    msg = String()
    msg.data = payload
    pub.publish(msg)

    used = [
        f"{c['ch']}:{c['joint']}"
        for c in servo_map.get("channels", [])
        if isinstance(c, dict) and c.get("joint")
    ]
    used_summary = ", ".join(used) if used else "(none)"
    node.get_logger().info(f"published servo_map from node labels; channels: {used_summary}")

    return payload


def main() -> None:
    if not NODE_NAME:
        raise SystemExit("NODE_NAME env is required (Downward API: spec.nodeName)")

    rclpy.init()
    node = Node("spot_node_label_config")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(String, TOPIC, qos)

    node.get_logger().info(
        f"starting; node={NODE_NAME} topic={TOPIC} label_prefix={LABEL_PREFIX}"
    )

    last_payload = None
    backoff = 1

    try:
        while rclpy.ok() and not _stop:
            try:
                obj = k8s_get_json(f"/api/v1/nodes/{NODE_NAME}")
                meta = obj.get("metadata", {}) if isinstance(obj, dict) else {}
                labels = meta.get("labels", {}) if isinstance(meta, dict) else {}
                resource_version = str(meta.get("resourceVersion", ""))

                if not resource_version:
                    raise RuntimeError("node resourceVersion is empty")

                if not isinstance(labels, dict):
                    labels = {}

                last_payload = publish_if_changed(node, pub, labels, last_payload)
                backoff = 1

                for event in k8s_watch_node_events(NODE_NAME, resource_version):
                    if _stop or not rclpy.ok():
                        break

                    etype = str(event.get("type", "")).upper()
                    eobj = event.get("object", {})
                    if not isinstance(eobj, dict):
                        continue

                    emeta = eobj.get("metadata", {})
                    if not isinstance(emeta, dict):
                        continue

                    elabels = emeta.get("labels", {})
                    if not isinstance(elabels, dict):
                        elabels = {}

                    if etype in ("ADDED", "MODIFIED"):
                        last_payload = publish_if_changed(node, pub, elabels, last_payload)
                    elif etype == "ERROR":
                        node.get_logger().error(f"watch error event: {event}")
                        break
            except Exception as e:
                node.get_logger().error(f"k8s watch failed: {e}; retry in {backoff}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, BACKOFF_MAX_SECONDS)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
