"""Bounded command/telemetry link for the companion MeArmSync sketch.

STATE reports the pulse last written to a servo, never a measured joint angle.
This module deliberately has no MuJoCo, NumPy, or pySerial import at load time.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

MAX_LINE = 160
PROTOCOL_VERSION = "1"


class LinkError(RuntimeError):
    """Transport, protocol, or firmware error; stop streaming and re-arm manually."""


def encode_frame(*tokens):
    words = tuple(str(token) for token in tokens)
    if not words or any(not word or not word.isascii()
                        or any(ord(c) < 33 or ord(c) > 126 or c == "*" for c in word)
                        for word in words):
        raise ValueError("Protocol tokens must be nonempty printable ASCII without spaces or '*'.")
    payload = " ".join(words).encode("ascii")
    checksum = 0
    for byte in payload:
        checksum ^= byte
    frame = payload + f"*{checksum:02X}\n".encode("ascii")
    if len(frame) > MAX_LINE:
        raise ValueError("Serial frame is too long.")
    return frame


def decode_frame(line):
    if len(line) > MAX_LINE:
        raise ValueError("Serial frame is too long.")
    line = bytes(line).rstrip(b"\r\n")
    if len(line) < 4 or line[-3:-2] != b"*":
        raise ValueError("Missing checksum.")
    payload, expected = line[:-3], line[-2:]
    if not payload or b"*" in payload or any(c < 32 or c > 126 for c in payload):
        raise ValueError("Invalid payload.")
    if any(c not in b"0123456789ABCDEFabcdef" for c in expected):
        raise ValueError("Invalid checksum digits.")
    checksum = 0
    for byte in payload:
        checksum ^= byte
    if checksum != int(expected, 16):
        raise ValueError("Checksum mismatch.")
    text = payload.decode("ascii")
    words = tuple(text.split(" "))
    if any(not word for word in words):
        raise ValueError("Invalid token spacing.")
    return words


class FrameReader:
    """Recover at the next newline after a corrupt or overlong USB frame."""

    def __init__(self):
        self.buffer = bytearray()
        self.dropping = False
        self.invalid_frames = 0

    def feed(self, data):
        frames = []
        for byte in data:
            if byte == 10:
                if not self.dropping:
                    try:
                        frames.append(decode_frame(self.buffer))
                    except ValueError:
                        self.invalid_frames += 1
                self.buffer.clear()
                self.dropping = False
            elif not self.dropping:
                self.buffer.append(byte)
                if len(self.buffer) >= MAX_LINE:
                    self.buffer.clear()
                    self.dropping = True
                    self.invalid_frames += 1
        return frames


@dataclass(frozen=True)
class ServoState:
    armed: bool
    reason: str
    pulses: tuple[int, int, int, int]


def _integer(word, lower, upper):
    if not word.isascii() or not word.isdecimal():
        raise LinkError("Firmware sent an invalid unsigned integer.")
    value = int(word)
    if not lower <= value <= upper:
        raise LinkError("Firmware sent an out-of-range integer.")
    return value


def _pulses(values):
    values = tuple(values)
    if len(values) != 4 or any(isinstance(v, bool) or not isinstance(v, int)
                               or not 900 <= v <= 2100 for v in values):
        raise ValueError("Four integer servo pulses in [900, 2100] microseconds are required.")
    return values


class SerialLink:
    """Request/response serial connection. Never sends ARM during connection.

    A single thread must own this link. Transactions use sequence numbers,
    checksums and short deadlines; there is no reconnect or automatic re-arm.
    """

    is_dry_run = False

    def __init__(self, stream, calibration, *, timeout=0.15):
        self.stream = stream
        self.calibration = calibration
        self.timeout = timeout
        self.reader = FrameReader()
        self.sequence = 0
        self.source = None
        self.firmware_armed = False
        self.closed = False

    @classmethod
    def open(cls, port, calibration, *, boot_seconds=2.0):
        try:
            import serial
        except ImportError as exc:
            raise LinkError("Install hardware support in this environment: "
                            "python -m pip install -r requirements-hardware.txt") from exc
        try:
            stream = serial.Serial(port, baudrate=calibration.baudrate,
                                   timeout=0.01, write_timeout=0.1)
        except (OSError, serial.SerialException) as exc:
            raise LinkError(f"Cannot open {port}: {exc}") from exc
        link = cls(stream, calibration)
        try:
            # Uno/Nano commonly reset when DTR changes on opening a USB port.
            time.sleep(boot_seconds)
            stream.reset_input_buffer()
            link.handshake()
        except BaseException:
            stream.close()
            raise
        return link

    def _request(self, command, *arguments):
        if self.closed:
            raise LinkError("Serial connection is closed.")
        self.sequence = (self.sequence + 1) % 65536
        seq = str(self.sequence)
        frame = encode_frame(command, seq, *arguments)
        try:
            if self.stream.write(frame) != len(frame):
                raise LinkError("Incomplete serial write.")
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                chunk = self.stream.read(min(512, max(1, self.stream.in_waiting)))
                for words in self.reader.feed(chunk):
                    if len(words) < 2 or words[1] != seq:
                        continue
                    if words[0] == "ERROR":
                        raise LinkError("Arduino rejected command: " + " ".join(words[2:]))
                    return words
        except (OSError, ValueError) as exc:
            raise LinkError(f"Serial communication failed: {exc}") from exc
        raise LinkError(f"No valid reply to {command} within {self.timeout:.2f}s; connection stopped.")

    def handshake(self):
        words = self._request("HELLO")
        if (len(words) != 6 or words[0] != "READY" or words[2] != PROTOCOL_VERSION
                or words[4] not in ("HOST", "LOCAL")):
            raise LinkError("This port does not speak MeArmSync protocol version 1.")
        if words[3].lower() != self.calibration.profile_id.lower():
            raise LinkError("Calibration ID mismatch. Export this JSON to calibration.h and upload the sketch again.")
        self.source = words[4]
        self.firmware_armed = bool(_integer(words[5], 0, 1))
        return self.get_state()

    def _ack(self, command, *arguments):
        words = self._request(command, *arguments)
        if len(words) != 3 or words[0] != "ACK" or words[2] != command:
            raise LinkError(f"Unexpected reply to {command}.")

    def get_state(self):
        words = self._request("STATE")
        if (len(words) != 8 or words[0] != "STATE"
                or words[3] not in ("BOOT", "ARM", "OK", "DISARM", "TIMEOUT", "ESTOP")):
            raise LinkError("Invalid Arduino STATE reply.")
        armed = bool(_integer(words[2], 0, 1))
        pulses = tuple(_integer(word, 900, 2100) for word in words[4:])
        # Validate against the same calibrated range as outgoing commands.
        self.calibration.to_degrees(pulses)
        self.firmware_armed = armed
        return ServoState(armed, words[3], pulses)

    def arm(self, pulses):
        if not self.calibration.calibrated:
            raise LinkError("Calibration is not commissioned; ARM is disabled.")
        if self.source != "HOST":
            raise LinkError("The Arduino is configured for LOCAL control; host ARM is disabled.")
        pulses = _pulses(pulses)
        self.calibration.to_degrees(pulses)
        self._ack("ARM", *pulses)
        self.firmware_armed = True

    def set_targets(self, pulses):
        pulses = _pulses(pulses)
        self.calibration.to_degrees(pulses)
        self._ack("SET", *pulses)

    def keep(self):
        self._ack("KEEP")

    def disarm(self):
        self._ack("DISARM")
        self.firmware_armed = False

    def estop(self):
        self._ack("ESTOP")
        self.firmware_armed = False

    def close(self):
        # Observer clients call close without sending any motion command.
        if not self.closed:
            self.stream.close()
            self.closed = True


class DryRunLink:
    """In-memory pulse/slew/watchdog emulator, with no serial port access."""

    is_dry_run = True
    source = "HOST"

    def __init__(self, calibration, *, clock=time.monotonic):
        self.calibration = calibration
        self.clock = clock
        self._last_tick = self._last_command = clock()
        self._current = list(map(float, calibration.home_pulses))
        self._target = list(self._current)
        self._armed = False
        self._reason = "BOOT"
        self._latched = False
        self.closed = False

    @property
    def firmware_armed(self):
        return self._armed

    def _tick(self):
        now = self.clock()
        dt = max(0.0, now - self._last_tick)
        if self._armed and now - self._last_command > self.calibration.watchdog_ms / 1000:
            self._armed = False
            self._reason = "TIMEOUT"
        if self._armed:
            for i, joint in enumerate(self.calibration.joints):
                step = joint.max_us_per_second * dt
                difference = self._target[i] - self._current[i]
                self._current[i] += max(-step, min(step, difference))
        self._last_tick = now

    def get_state(self):
        self._tick()
        return ServoState(self._armed, self._reason, tuple(int(math.floor(p + 0.5)) for p in self._current))

    def arm(self, pulses):
        if self._latched:
            raise LinkError("ESTOP is latched; restart the dry run.")
        if self._armed:
            raise LinkError("Already armed.")
        pulses = _pulses(pulses)
        self.calibration.to_degrees(pulses)
        self._current = list(map(float, pulses))
        self._target = list(self._current)
        self._armed, self._reason = True, "ARM"
        self._last_tick = self._last_command = self.clock()

    def set_targets(self, pulses):
        self._tick()
        if not self._armed:
            raise LinkError("Not armed; explicit ARM is required.")
        pulses = _pulses(pulses)
        self.calibration.to_degrees(pulses)
        self._target = list(pulses)
        self._last_command = self.clock()
        self._reason = "OK"

    def keep(self):
        self._tick()
        self._last_command = self.clock()

    def disarm(self):
        self._tick()
        self._armed, self._reason = False, "DISARM"

    def estop(self):
        self._tick()
        self._armed, self._reason, self._latched = False, "ESTOP", True

    def close(self):
        self.closed = True


class Synchronizer:
    """One-way master selection with a latched host fault and no auto re-arm."""

    def __init__(self, arm, calibration, link, direction="sim-to-hardware", hz=20):
        if direction not in ("sim-to-hardware", "hardware-to-sim"):
            raise ValueError("Unknown synchronization direction.")
        if not math.isfinite(hz) or not 5 <= hz <= 50:
            raise ValueError("Synchronization rate must be between 5 and 50 Hz.")
        if hz < 3000 / calibration.watchdog_ms:
            raise ValueError("Synchronization rate must provide at least three updates per watchdog interval.")
        self.arm_controller = arm
        self.calibration = calibration
        self.link = link
        self.direction = direction
        self.period = 1 / hz
        self.enabled = False
        self.fault = None
        self.state = None
        self.last_update = -math.inf
        self.applied_degrees = None

    def arm(self):
        if self.direction != "sim-to-hardware":
            raise ValueError("hardware-to-sim is read-only; use the Arduino's local enable control.")
        if self.enabled:
            raise ValueError("Already armed. DISARM before requesting another ARM.")
        if self.fault:
            raise LinkError("Connection fault is latched. Restart the bridge before arming again.")
        # Always start from the calibrated HOME, never from a slider moved while disarmed.
        values = self.calibration.home_degrees
        self.arm_controller.set_joint_targets(**dict(zip(
            ("yaw_deg", "shoulder_deg", "forearm_deg", "grip_deg"), values)))
        try:
            self.link.arm(self.calibration.home_pulses)
        except (LinkError, ValueError, OSError) as exc:
            # ARM may have reached the board even if its ACK was lost.
            self._stop_on_fault(str(exc))
        self.enabled = True
        self.last_update = -math.inf

    def disarm(self):
        if self.direction != "sim-to-hardware":
            raise ValueError("Observer mode cannot send DISARM; use the local hardware control.")
        self.enabled = False
        try:
            self.link.disarm()
        except (LinkError, ValueError, OSError) as exc:
            self._stop_on_fault(str(exc))

    def estop(self):
        # Emergency stop is available even when observing; it is an explicit user action.
        self.enabled = False
        self.fault = "ESTOP latched. Reset the Arduino and restart this bridge."
        self.link.estop()

    def _stop_on_fault(self, message):
        self.enabled = False
        self.fault = message
        if self.direction == "sim-to-hardware":
            try:
                self.link.disarm()
            except (LinkError, OSError, ValueError):
                pass  # The independent board watchdog handles a broken cable.
        raise LinkError(message)

    def update(self, now=None):
        now = time.monotonic() if now is None else now
        if self.fault or now - self.last_update < self.period:
            return
        self.last_update = now
        try:
            if self.direction == "sim-to-hardware" and self.enabled:
                self.link.set_targets(self.calibration.to_pulses(self.arm_controller.target_degrees))
            state = self.link.get_state()
            self.state = state
            self.applied_degrees = self.calibration.to_degrees(state.pulses)
            if self.direction == "hardware-to-sim":
                self.arm_controller.set_joint_targets(**dict(zip(
                    ("yaw_deg", "shoulder_deg", "forearm_deg", "grip_deg"), self.applied_degrees)))
            elif self.enabled and not state.armed:
                raise LinkError(f"Arduino disarmed ({state.reason}); explicit restart is required.")
        except (LinkError, ValueError, OSError) as exc:
            self._stop_on_fault(str(exc))

    def close(self):
        try:
            if self.direction == "sim-to-hardware":
                self.link.disarm()
        except (LinkError, OSError, ValueError):
            pass
        finally:
            self.enabled = False
            self.link.close()
