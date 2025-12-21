import fcntl
import json
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SERVO_MAP_TOPIC = (
    os.environ.get("SERVO_MAP_TOPIC", "/spot/config/servo_map").strip()
    or "/spot/config/servo_map"
)
CMD_TOPIC = os.environ.get("CMD_TOPIC", "/spot/cmd/servo").strip() or "/spot/cmd/servo"
STATUS_TOPIC = (
    os.environ.get("STATUS_TOPIC", "/spot/state/servo").strip() or "/spot/state/servo"
)

PWM_HZ = float(os.environ.get("PWM_HZ", "50"))
UPDATE_HZ = float(os.environ.get("UPDATE_HZ", "50"))
MAX_SLEW_US_PER_S = float(os.environ.get("MAX_SLEW_US_PER_S", "1200"))

START_ARMED = str(os.environ.get("START_ARMED", "0")).strip().lower() in (
    "1",
    "true",
    "yes",
    "y",
    "on",
)
DISARM_FULL_OFF = str(os.environ.get("DISARM_FULL_OFF", "1")).strip().lower() in (
    "1",
    "true",
    "yes",
    "y",
    "on",
)

I2C_DEVICE_TEMPLATE = (
    os.environ.get("I2C_DEVICE_TEMPLATE", "/dev/i2c-{bus}").strip() or "/dev/i2c-{bus}"
)

STATUS_PUBLISH_SECONDS = float(os.environ.get("STATUS_PUBLISH_SECONDS", "1.0"))

_stop = False


def _handle_stop(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)


def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _parse_int(raw, default: int) -> int:
    try:
        return int(str(raw).strip(), 0)
    except Exception:
        return default


def _parse_bool(raw, default: bool = False) -> bool:
    if raw is None:
        return default
    val = str(raw).strip().lower()
    if val in ("1", "true", "yes", "y", "on"):
        return True
    if val in ("0", "false", "no", "n", "off"):
        return False
    return default


def _parse_servo_map(raw_json: str) -> dict:
    try:
        obj = json.loads(raw_json)
    except Exception:
        return {}
    return obj if isinstance(obj, dict) else {}


def _sanitize_triplet(min_us: int, center_us: int, max_us: int):
    vals = [min_us, center_us, max_us]
    if any(not isinstance(v, int) for v in vals):
        return 1000, 1500, 2000

    min_us = _clamp(min_us, 500, 2500)
    center_us = _clamp(center_us, 500, 2500)
    max_us = _clamp(max_us, 500, 2500)

    if min_us > max_us:
        min_us, max_us = max_us, min_us

    center_us = _clamp(center_us, min_us, max_us)
    if min_us == max_us:
        min_us = max(500, min_us - 1)
        max_us = min(2500, max_us + 1)
        center_us = (min_us + max_us) // 2

    if center_us == min_us:
        center_us = min_us + 1
    if center_us == max_us:
        center_us = max_us - 1

    return min_us, center_us, max_us


def _norm_to_us(
    norm: float, min_us: int, center_us: int, max_us: int, invert: bool
) -> int:
    if invert:
        norm = -norm
    norm = float(_clamp(norm, -1.0, 1.0))

    if norm >= 0.0:
        span = max_us - center_us
        us = center_us + norm * span
    else:
        span = center_us - min_us
        us = center_us + norm * span

    return int(round(_clamp(us, min_us, max_us)))


def _us_to_norm(
    us: int, min_us: int, center_us: int, max_us: int, invert: bool
) -> float:
    us = int(_clamp(us, min_us, max_us))
    if us == center_us:
        norm = 0.0
    elif us > center_us:
        denom = float(max(1, max_us - center_us))
        norm = (us - center_us) / denom
    else:
        denom = float(max(1, center_us - min_us))
        norm = (us - center_us) / denom

    norm = float(_clamp(norm, -1.0, 1.0))
    return -norm if invert else norm


class I2CDev:
    I2C_SLAVE = 0x0703

    def __init__(self, bus: int, address: int, device_template: str = "/dev/i2c-{bus}"):
        self._path = device_template.format(bus=bus)
        self._fd = os.open(self._path, os.O_RDWR)
        fcntl.ioctl(self._fd, self.I2C_SLAVE, address)

    @property
    def path(self) -> str:
        return self._path

    def write(self, register: int, data: bytes) -> None:
        os.write(self._fd, bytes([register & 0xFF]) + (data or b""))

    def read(self, register: int, length: int) -> bytes:
        os.write(self._fd, bytes([register & 0xFF]))
        return os.read(self._fd, length)

    def close(self) -> None:
        try:
            os.close(self._fd)
        except Exception:
            pass


class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    AI = 0x20
    SLEEP = 0x10
    RESTART = 0x80

    OUTDRV = 0x04

    def __init__(self, bus: int, address: int, device_template: str):
        self._bus = bus
        self._address = address
        self._dev = I2CDev(bus, address, device_template=device_template)
        self._freq_hz = None
        self._init_chip()

    @property
    def bus(self) -> int:
        return self._bus

    @property
    def address(self) -> int:
        return self._address

    @property
    def freq_hz(self):
        return self._freq_hz

    @property
    def i2c_path(self) -> str:
        return self._dev.path

    def close(self):
        self._dev.close()

    def _read_u8(self, reg: int) -> int:
        return int(self._dev.read(reg, 1)[0])

    def _write_u8(self, reg: int, val: int) -> None:
        self._dev.write(reg, bytes([val & 0xFF]))

    def _init_chip(self):
        self._write_u8(self.MODE1, self.AI)
        self._write_u8(self.MODE2, self.OUTDRV)

    def set_pwm_freq(self, freq_hz: float, osc_hz: float = 25_000_000.0) -> None:
        freq_hz = float(_clamp(freq_hz, 24.0, 1526.0))

        prescale = int(round(osc_hz / (4096.0 * freq_hz) - 1.0))
        prescale = int(_clamp(prescale, 3, 255))

        old_mode = self._read_u8(self.MODE1)
        sleep_mode = (old_mode & 0x7F) | self.SLEEP
        self._write_u8(self.MODE1, sleep_mode)
        self._write_u8(self.PRESCALE, prescale)
        self._write_u8(self.MODE1, old_mode)
        time.sleep(0.005)
        self._write_u8(self.MODE1, old_mode | self.RESTART | self.AI)

        self._freq_hz = freq_hz

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        channel = int(channel)
        if channel < 0 or channel > 15:
            return
        on = int(_clamp(on, 0, 4095))
        off = int(_clamp(off, 0, 4095))
        reg = self.LED0_ON_L + 4 * channel
        data = bytes(
            [
                on & 0xFF,
                (on >> 8) & 0x0F,
                off & 0xFF,
                (off >> 8) & 0x0F,
            ]
        )
        self._dev.write(reg, data)

    def set_full_off(self, channel: int) -> None:
        channel = int(channel)
        if channel < 0 or channel > 15:
            return
        reg = self.LED0_ON_L + 4 * channel
        self._dev.write(reg, bytes([0x00, 0x00, 0x00, 0x10]))

    def set_pulse_us(self, channel: int, pulse_us: int) -> None:
        if not self._freq_hz:
            raise RuntimeError("PCA9685 frequency is not set")

        pulse_us = int(pulse_us)
        steps = int(round(pulse_us * self._freq_hz * 4096.0 / 1_000_000.0))
        steps = int(_clamp(steps, 0, 4095))
        self.set_pwm(channel, 0, steps)


class SpotServoDriver(Node):
    def __init__(self):
        super().__init__("spot_servo_driver")

        qos_cfg = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._servo_map_sub = self.create_subscription(
            String, SERVO_MAP_TOPIC, self._on_servo_map, qos_cfg
        )
        self._cmd_sub = self.create_subscription(String, CMD_TOPIC, self._on_cmd, 10)

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self._armed = START_ARMED
        self._estop = False

        self._pca = None
        self._pca_bus = None
        self._pca_address = None

        self._joints = {}
        self._targets_norm = {}
        self._current_us = {}

        self._last_tick = time.monotonic()
        self._last_status = 0.0

        self._timer = self.create_timer(1.0 / max(1.0, UPDATE_HZ), self._tick)

        self.get_logger().info(
            f"starting; map={SERVO_MAP_TOPIC} cmd={CMD_TOPIC} pwm_hz={PWM_HZ:.1f} "
            f"update_hz={UPDATE_HZ:.1f} slew={MAX_SLEW_US_PER_S:.0f}us/s "
            f"armed={'true' if self._armed else 'false'}"
        )

    def _publish_status(self):
        now = time.monotonic()
        if now - self._last_status < max(0.1, STATUS_PUBLISH_SECONDS):
            return
        self._last_status = now

        joints = {}
        for joint, cfg in self._joints.items():
            ch = cfg.get("ch")
            joints[joint] = {
                "ch": ch,
                "target_norm": self._targets_norm.get(joint, 0.0),
                "current_us": int(
                    round(self._current_us.get(ch, cfg.get("center_us", 1500)))
                ),
            }

        payload = {
            "armed": self._armed,
            "estop": self._estop,
            "pca9685": {
                "bus": self._pca_bus,
                "address": self._pca_address,
                "freq_hz": self._pca.freq_hz if self._pca else None,
            },
            "joints": joints,
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _set_all_off(self):
        if not self._pca:
            return
        for ch in range(16):
            try:
                self._pca.set_full_off(ch)
            except Exception:
                pass

    def _on_servo_map(self, msg: String):
        raw = msg.data or ""
        obj = _parse_servo_map(raw)

        pca = obj.get("pca9685", {})
        if not isinstance(pca, dict):
            pca = {}

        bus = _parse_int(pca.get("i2c_bus", 1), 1)
        address = _parse_int(pca.get("address", 0x40), 0x40)

        channels = obj.get("channels", [])
        if not isinstance(channels, list):
            channels = []

        new_joints = {}
        for entry in channels:
            if not isinstance(entry, dict):
                continue

            joint = str(entry.get("joint", "")).strip()
            if not joint:
                continue

            ch = _parse_int(entry.get("ch", -1), -1)
            if ch < 0 or ch > 15:
                continue

            min_us = _parse_int(entry.get("min_us", 1000), 1000)
            center_us = _parse_int(entry.get("center_us", 1500), 1500)
            max_us = _parse_int(entry.get("max_us", 2000), 2000)
            min_us, center_us, max_us = _sanitize_triplet(min_us, center_us, max_us)

            invert = _parse_bool(entry.get("invert", False), False)

            new_joints[joint] = {
                "ch": ch,
                "min_us": min_us,
                "center_us": center_us,
                "max_us": max_us,
                "invert": invert,
            }

        need_reconnect = (
            (bus != self._pca_bus)
            or (address != self._pca_address)
            or (self._pca is None)
        )
        if need_reconnect:
            try:
                if self._pca is not None:
                    self._pca.close()
            except Exception:
                pass

            try:
                self._pca = PCA9685(
                    bus=bus, address=address, device_template=I2C_DEVICE_TEMPLATE
                )
                self._pca.set_pwm_freq(PWM_HZ)
                self._pca_bus = bus
                self._pca_address = address
                self.get_logger().info(
                    f"connected to PCA9685 bus={bus} address={hex(address)} "
                    f"(i2c={self._pca.i2c_path}) pwm_hz={PWM_HZ:.1f}"
                )
            except Exception as e:
                self._pca = None
                self._pca_bus = bus
                self._pca_address = address
                self.get_logger().error(
                    f"failed to init PCA9685 (bus={bus} addr={hex(address)}): {e}"
                )

        self._joints = new_joints

        for joint in list(self._targets_norm.keys()):
            if joint not in self._joints:
                self._targets_norm.pop(joint, None)

        for joint in self._joints.keys():
            self._targets_norm.setdefault(joint, 0.0)

        if DISARM_FULL_OFF and not self._armed:
            self._set_all_off()

        used_summary = ", ".join(
            f"{cfg['ch']}:{joint}" for joint, cfg in sorted(self._joints.items())
        )
        self.get_logger().info(
            f"servo_map updated; joints={len(self._joints)} channels={used_summary or '(none)'}"
        )

    def _on_cmd(self, msg: String):
        raw = (msg.data or "").strip()
        if not raw:
            return

        try:
            cmd = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"invalid cmd json: {e}")
            return

        if not isinstance(cmd, dict):
            self.get_logger().error("invalid cmd payload: expected object")
            return

        ctype = str(cmd.get("cmd", "")).strip().lower()
        if ctype == "arm":
            value = _parse_bool(cmd.get("value", False), False)
            if value and self._estop:
                self.get_logger().error("arm rejected: estop=true (clear estop first)")
                return
            self._armed = value
            if DISARM_FULL_OFF and not self._armed:
                self._set_all_off()
            self.get_logger().info(f"armed={'true' if self._armed else 'false'}")
            return

        if ctype == "estop":
            value = _parse_bool(cmd.get("value", False), False)
            self._estop = value
            if self._estop:
                self._armed = False
                if DISARM_FULL_OFF:
                    self._set_all_off()
                self.get_logger().warn("ESTOP engaged; outputs disabled")
            else:
                self.get_logger().info("estop cleared; still disarmed")
            return

        if ctype == "home":
            for joint in self._joints.keys():
                self._targets_norm[joint] = 0.0
            self.get_logger().info("targets set to home (0.0)")
            return

        if ctype == "set":
            targets = cmd.get("targets", {})
            mode = str(cmd.get("mode", "norm")).strip().lower() or "norm"
            if not isinstance(targets, dict):
                self.get_logger().error("cmd.set: targets must be object")
                return

            unknown = []
            for joint, val in targets.items():
                joint = str(joint).strip()
                if joint not in self._joints:
                    unknown.append(joint)
                    continue

                cfg = self._joints[joint]
                if mode == "us":
                    us = _parse_int(val, cfg.get("center_us", 1500))
                    norm = _us_to_norm(
                        us,
                        cfg["min_us"],
                        cfg["center_us"],
                        cfg["max_us"],
                        cfg["invert"],
                    )
                    self._targets_norm[joint] = norm
                else:
                    try:
                        norm = float(val)
                    except Exception:
                        continue
                    self._targets_norm[joint] = float(_clamp(norm, -1.0, 1.0))

            if unknown:
                self.get_logger().warn(f"cmd.set: unknown joints: {', '.join(unknown)}")
            return

        self.get_logger().warn(f"unknown cmd: {ctype or '(empty)'}")

    def _tick(self):
        if not self._pca or not self._joints:
            self._publish_status()
            return

        if self._estop or not self._armed:
            self._publish_status()
            return

        now = time.monotonic()
        dt = max(0.001, now - self._last_tick)
        self._last_tick = now

        max_delta = MAX_SLEW_US_PER_S * dt
        max_delta = float(_clamp(max_delta, 1.0, 10_000.0))

        try:
            for joint, cfg in self._joints.items():
                ch = cfg["ch"]
                target_norm = float(self._targets_norm.get(joint, 0.0))
                target_us = _norm_to_us(
                    target_norm,
                    cfg["min_us"],
                    cfg["center_us"],
                    cfg["max_us"],
                    cfg["invert"],
                )

                current = float(self._current_us.get(ch, target_us))
                delta = target_us - current
                if abs(delta) <= max_delta:
                    new_us = float(target_us)
                else:
                    new_us = current + math.copysign(max_delta, delta)

                new_us = float(_clamp(new_us, cfg["min_us"], cfg["max_us"]))
                self._current_us[ch] = new_us
                self._pca.set_pulse_us(ch, int(round(new_us)))
        except Exception as e:
            self.get_logger().error(f"tick failed: {e}")

        self._publish_status()

    def shutdown(self) -> None:
        if DISARM_FULL_OFF:
            self._set_all_off()
        try:
            if self._pca is not None:
                self._pca.close()
        except Exception:
            pass
        self._pca = None


def main() -> None:
    rclpy.init()
    node = SpotServoDriver()

    try:
        while rclpy.ok() and not _stop:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
