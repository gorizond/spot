#!/usr/bin/env python3
import os
import time

import RPi.GPIO as GPIO
from RPLCD.gpio import CharLCD

GPIO.setwarnings(False)

PIN_RS = 25
PIN_E = 24
PINS_DATA = [23, 17, 18, 22]

LCD_COLS = 16
LCD_ROWS = 2


def read_first(path_a, path_b):
    for path in (path_a, path_b):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return handle.read().strip()
        except OSError:
            continue
    return None


def read_temp_c():
    raw = read_first(
        "/host/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone0/temp",
    )
    if not raw:
        return None
    try:
        value = int(raw)
    except ValueError:
        return None
    return value / 1000.0


def read_uptime_seconds():
    raw = read_first("/host/proc/uptime", "/proc/uptime")
    if not raw:
        return None
    try:
        return float(raw.split()[0])
    except (ValueError, IndexError):
        return None


def format_uptime(seconds):
    if seconds is None:
        return "--:--"
    seconds = max(0, int(seconds))
    hours = min(seconds // 3600, 99)
    minutes = (seconds % 3600) // 60
    return f"{hours:02d}:{minutes:02d}"


def format_temp(temp_c):
    if temp_c is None:
        return "--.-C"
    return f"{temp_c:4.1f}C"


def main():
    lcd = CharLCD(
        numbering_mode=GPIO.BCM,
        cols=LCD_COLS,
        rows=LCD_ROWS,
        pin_rs=PIN_RS,
        pin_e=PIN_E,
        pins_data=PINS_DATA,
    )

    lcd.clear()

    while True:
        temp_c = read_temp_c()
        uptime = format_uptime(read_uptime_seconds())

        line1 = f"Temp {format_temp(temp_c)}"
        line2 = f"Uptime {uptime}"

        lcd.cursor_pos = (0, 0)
        lcd.write_string(line1.ljust(LCD_COLS)[:LCD_COLS])
        lcd.cursor_pos = (1, 0)
        lcd.write_string(line2.ljust(LCD_COLS)[:LCD_COLS])
        time.sleep(2)


if __name__ == "__main__":
    main()
