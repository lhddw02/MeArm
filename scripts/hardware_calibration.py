"""Validated, measured joint-angle / servo-pulse calibration for MeArm Sync.

This module has no serial or simulator dependencies and never moves hardware.
The example profile is deliberately uncalibrated: its numbers are illustrations,
not verified pulse limits for a particular servo or linkage.
"""
from __future__ import annotations

from bisect import bisect_right
from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path
import zlib


JOINT_NAMES = ("yaw", "shoulder", "forearm", "grip")
MODEL_LIMITS_DEG = ((-45.0, 45.0), (45.0, 135.0), (-45.0, 45.0),
                    (0.0, math.degrees(0.7)))
PULSE_LIMITS_US = (900, 2100)
_EPSILON = 1e-9


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} must be a finite number.")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a finite number.") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number.")
    return result


def _integer(value, label):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer.")
    return value


def _sequence(value, label):
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{label} must be an array.")
    try:
        return tuple(value)
    except TypeError as exc:
        raise ValueError(f"{label} must be an array.") from exc


def _fields(value, required, label):
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object.")
    missing = set(required) - set(value)
    unexpected = set(value) - set(required)
    if missing or unexpected:
        details = []
        if missing:
            details.append("missing " + ", ".join(sorted(missing)))
        if unexpected:
            details.append("unknown " + ", ".join(sorted(map(str, unexpected))))
        raise ValueError(f"{label}: {'; '.join(details)}.")


def _interpolate(value, xs, ys, label):
    """Interpolate within measured anchors only; tolerate numeric roundoff."""
    value = _number(value, label)
    if value < xs[0] - _EPSILON or value > xs[-1] + _EPSILON:
        raise ValueError(f"{label} {value:g} is outside calibrated range "
                         f"[{xs[0]:g}, {xs[-1]:g}].")
    value = min(max(value, xs[0]), xs[-1])
    index = min(max(bisect_right(xs, value) - 1, 0), len(xs) - 2)
    fraction = (value - xs[index]) / (xs[index + 1] - xs[index])
    return ys[index] + fraction * (ys[index + 1] - ys[index])


@dataclass(frozen=True)
class JointCalibration:
    name: str
    pin: int
    model_deg: tuple[float, ...]
    pulse_us: tuple[int, ...]
    home_deg: float
    max_us_per_second: float

    @property
    def min_deg(self):
        return self.model_deg[0]

    @property
    def max_deg(self):
        return self.model_deg[-1]

    @property
    def min_us(self):
        return min(self.pulse_us)

    @property
    def max_us(self):
        return max(self.pulse_us)

    @property
    def home_us(self):
        return self.to_pulse(self.home_deg)

    def to_pulse(self, degrees):
        pulse = _interpolate(degrees, self.model_deg, self.pulse_us,
                             f"{self.name} angle (degrees)")
        # Servo.writeMicroseconds takes whole microseconds. All values are > 0.
        return int(math.floor(pulse + 0.5))

    def to_degrees(self, pulse_us):
        pulses, degrees = self.pulse_us, self.model_deg
        if pulses[0] > pulses[-1]:
            pulses, degrees = pulses[::-1], degrees[::-1]
        return _interpolate(pulse_us, pulses, degrees,
                            f"{self.name} pulse (microseconds)")

    def to_dict(self):
        return {"name": self.name, "pin": self.pin,
                "model_deg": list(self.model_deg), "pulse_us": list(self.pulse_us),
                "home_deg": self.home_deg,
                "max_us_per_second": self.max_us_per_second}


@dataclass(frozen=True)
class Calibration:
    calibrated: bool
    baudrate: int
    watchdog_ms: int
    joints: tuple[JointCalibration, ...]

    @classmethod
    def load(cls, path):
        """Read and validate JSON; loading does not open a serial port."""
        try:
            source = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"Cannot read calibration {path}: {exc}") from exc
        return cls.from_dict(source)

    @classmethod
    def from_dict(cls, source):
        _fields(source, ("version", "calibrated", "baudrate", "watchdog_ms", "joints"),
                "Calibration")
        if _integer(source["version"], "version") != 1:
            raise ValueError("Unsupported calibration version; expected 1.")
        if not isinstance(source["calibrated"], bool):
            raise ValueError("calibrated must be true or false.")
        baudrate = _integer(source["baudrate"], "baudrate")
        if baudrate != 115200:
            raise ValueError("baudrate must be 115200 for this firmware.")
        watchdog = _integer(source["watchdog_ms"], "watchdog_ms")
        if not 200 <= watchdog <= 2000:
            raise ValueError("watchdog_ms must be between 200 and 2000.")
        source_joints = _sequence(source["joints"], "joints")
        if len(source_joints) != 4:
            raise ValueError("joints must contain exactly yaw, shoulder, forearm, grip in that order.")

        joints = []
        pins = set()
        for index, entry in enumerate(source_joints):
            label = f"joints[{index}]"
            _fields(entry, ("name", "pin", "model_deg", "pulse_us", "home_deg",
                            "max_us_per_second"), label)
            name = entry["name"]
            if name != JOINT_NAMES[index]:
                raise ValueError("joints must contain exactly yaw, shoulder, forearm, grip in that order.")
            pin = _integer(entry["pin"], f"{name}.pin")
            if not 2 <= pin <= 13 or pin in pins:
                raise ValueError(f"{name}.pin must be a unique Uno/Nano pin from 2 to 13 (0/1 are serial).")
            pins.add(pin)
            angles = tuple(_number(v, f"{name}.model_deg") for v in
                           _sequence(entry["model_deg"], f"{name}.model_deg"))
            pulses = tuple(_integer(v, f"{name}.pulse_us") for v in
                           _sequence(entry["pulse_us"], f"{name}.pulse_us"))
            if len(angles) < 3 or len(angles) != len(pulses):
                raise ValueError(f"{name} needs at least three paired measured angle/pulse anchors.")
            if not all(a < b for a, b in zip(angles, angles[1:])):
                raise ValueError(f"{name}.model_deg must be strictly increasing.")
            low, high = MODEL_LIMITS_DEG[index]
            if angles[0] < low or angles[-1] > high:
                raise ValueError(f"{name}.model_deg must stay within simulation range [{low:g}, {high:g}].")
            increasing = all(a < b for a, b in zip(pulses, pulses[1:]))
            decreasing = all(a > b for a, b in zip(pulses, pulses[1:]))
            if not (increasing or decreasing):
                raise ValueError(f"{name}.pulse_us must be strictly increasing or strictly decreasing.")
            if min(pulses) < PULSE_LIMITS_US[0] or max(pulses) > PULSE_LIMITS_US[1]:
                raise ValueError(f"{name}.pulse_us must stay within commissioning bounds 900..2100 microseconds.")
            home = _number(entry["home_deg"], f"{name}.home_deg")
            if not angles[0] <= home <= angles[-1]:
                raise ValueError(f"{name}.home_deg must be within its measured angle anchors.")
            rate = _number(entry["max_us_per_second"], f"{name}.max_us_per_second")
            if not 0 < rate <= 1000:
                raise ValueError(f"{name}.max_us_per_second must be greater than 0 and at most 1000.")
            joints.append(JointCalibration(name, pin, angles, pulses, home, rate))
        return cls(source["calibrated"], baudrate, watchdog, tuple(joints))

    def to_dict(self):
        return {"version": 1, "calibrated": self.calibrated,
                "baudrate": self.baudrate, "watchdog_ms": self.watchdog_ms,
                "joints": [joint.to_dict() for joint in self.joints]}

    @property
    def profile_id(self):
        """Eight lowercase hex digits identifying all normalized profile fields."""
        serialized = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"),
                                allow_nan=False).encode("utf-8")
        return f"{zlib.crc32(serialized) & 0xffffffff:08x}"

    @property
    def home_degrees(self):
        return tuple(joint.home_deg for joint in self.joints)

    @property
    def home_pulses(self):
        return tuple(joint.home_us for joint in self.joints)

    def to_pulses(self, degrees):
        values = _sequence(degrees, "Joint angles")
        if len(values) != 4:
            raise ValueError("Joint angles must contain yaw, shoulder, forearm, grip.")
        return tuple(joint.to_pulse(angle) for joint, angle in zip(self.joints, values))

    def to_degrees(self, pulses):
        values = _sequence(pulses, "Servo pulses")
        if len(values) != 4:
            raise ValueError("Servo pulses must contain yaw, shoulder, forearm, grip.")
        return tuple(joint.to_degrees(pulse) for joint, pulse in zip(self.joints, values))

    def firmware_header(self):
        def ints(attribute):
            return ", ".join(str(getattr(joint, attribute)) for joint in self.joints)

        rates = ", ".join(f"{joint.max_us_per_second:.9f}f" for joint in self.joints)
        return (
            "// Generated by scripts/calibrate_hardware.py; edit the JSON profile, then export again.\n"
            "// Order: yaw, shoulder absolute elevation, forearm absolute elevation, grip.\n"
            "// Uncalibrated examples cannot arm. Pulse bounds are measured anchors, not servo ratings.\n"
            "#pragma once\n#include <Arduino.h>\n\n"
            f'#define MEARM_PROFILE_ID "{self.profile_id}"\n'
            f"#define MEARM_CALIBRATED {str(self.calibrated).lower()}\n"
            "#ifndef MEARM_SOURCE_LOCAL\n#define MEARM_SOURCE_LOCAL false\n#endif\n\n"
            f"static const uint8_t MEARM_PINS[4] = {{{ints('pin')}}};\n"
            f"static const int MEARM_MIN_US[4] = {{{ints('min_us')}}};\n"
            f"static const int MEARM_MAX_US[4] = {{{ints('max_us')}}};\n"
            f"static const int MEARM_HOME_US[4] = {{{ints('home_us')}}};\n"
            f"static const float MEARM_MAX_US_PER_SECOND[4] = {{{rates}}};\n"
            f"static const unsigned long MEARM_WATCHDOG_MS = {self.watchdog_ms}UL;\n"
        )
