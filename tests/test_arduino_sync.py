"""Protocol failures and master selection, using virtual servos only."""
from copy import deepcopy
from pathlib import Path
import unittest

import numpy as np

from scripts.arduino_sync import (DryRunLink, FrameReader, LinkError, SerialLink,
                                 Synchronizer, decode_frame, encode_frame)
from scripts.hardware_calibration import Calibration
from scripts.sync_arduino import handle_sync_command, load_model

ROOT = Path(__file__).resolve().parents[1]


class FakeSerial:
    """Byte-level firmware responder; no pySerial or OS device involved."""

    def __init__(self, calibration):
        self.calibration = calibration
        self.incoming = bytearray()
        self.commands = []
        self.armed = False
        self.pulses = calibration.home_pulses
        self.profile_id = calibration.profile_id
        self.source = "HOST"
        self.closed = False
        self.drop_ack = False
        self.corrupt_first = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def write(self, frame):
        words = decode_frame(frame)
        self.commands.append(words)
        name, seq = words[:2]
        if name == "HELLO":
            reply = ("READY", seq, "1", self.profile_id, self.source, int(self.armed))
        elif name == "STATE":
            reply = ("STATE", seq, int(self.armed), "OK" if self.armed else "BOOT", *self.pulses)
        else:
            if name == "ARM":
                self.armed = True
                self.pulses = tuple(map(int, words[2:]))
            elif name == "SET":
                self.pulses = tuple(map(int, words[2:]))
            elif name in ("DISARM", "ESTOP"):
                self.armed = False
            reply = ("ACK", seq, name)
        if self.corrupt_first:
            self.incoming.extend(b"bad frame*FF\n")
            self.corrupt_first = False
        if not self.drop_ack:
            self.incoming.extend(encode_frame(*reply))
        return len(frame)

    def read(self, count):
        # Fragment every reply to exercise partial USB reads.
        count = min(count, 7)
        chunk = bytes(self.incoming[:count])
        del self.incoming[:count]
        return chunk

    def close(self):
        self.closed = True


class ArduinoSyncTests(unittest.TestCase):
    def setUp(self):
        self.calibration = Calibration.load(ROOT / "config/hardware_calibration.example.json")
        self.now = 0.0

    def make_sync(self, direction="sim-to-hardware"):
        model, data, arm = load_model(self.calibration, "arm")
        link = DryRunLink(self.calibration, clock=lambda: self.now)
        return model, data, arm, link, Synchronizer(arm, self.calibration, link, direction)

    def test_frames_fragmentation_corruption_and_bounded_resynchronization(self):
        frame = encode_frame("SET", 65535, 1500, 1266, 1472, 1850)
        self.assertEqual(decode_frame(frame), ("SET", "65535", "1500", "1266", "1472", "1850"))
        reader = FrameReader()
        self.assertEqual(reader.feed(frame[:5]), [])
        self.assertEqual(reader.feed(frame[5:]), [decode_frame(frame)])
        self.assertEqual(reader.feed(b"nonsense*FF\n"), [])
        self.assertEqual(reader.feed(b"x" * 10000), [])
        self.assertLessEqual(len(reader.buffer), 160)
        self.assertEqual(reader.feed(b"\n" + frame), [decode_frame(frame)])
        self.assertEqual(reader.invalid_frames, 2)
        for invalid in (b"HELLO 1*GG\n", b"HELLO 1*00\n", b"HELLO  1*00\n"):
            with self.assertRaises(ValueError):
                decode_frame(invalid)
        for words in (("SET 1",), ("SET", "*"), ("SET", "\n"), ("SET", "a" * 200)):
            with self.assertRaises(ValueError):
                encode_frame(*words)

    def test_serial_handshake_checks_identity_and_never_arms(self):
        serial = FakeSerial(self.calibration)
        serial.corrupt_first = True
        link = SerialLink(serial, self.calibration, timeout=0.01)
        state = link.handshake()
        self.assertEqual([words[0] for words in serial.commands], ["HELLO", "STATE"])
        self.assertFalse(state.armed)
        self.assertEqual(state.pulses, self.calibration.home_pulses)
        self.assertEqual(link.reader.invalid_frames, 1)
        serial.profile_id = "00000000"
        with self.assertRaisesRegex(LinkError, "Calibration ID mismatch"):
            link.handshake()

    def test_live_arm_gate_and_round_trip_with_commissioned_virtual_profile(self):
        serial = FakeSerial(self.calibration)
        link = SerialLink(serial, self.calibration)
        link.handshake()
        with self.assertRaisesRegex(LinkError, "not commissioned"):
            link.arm(self.calibration.home_pulses)
        self.assertFalse(serial.armed)
        profile = self.calibration.to_dict()
        profile["calibrated"] = True
        calibration = Calibration.from_dict(profile)
        serial = FakeSerial(calibration)
        link = SerialLink(serial, calibration)
        link.handshake()
        link.arm(calibration.home_pulses)
        target = calibration.to_pulses((2, 92, -2, 15))
        link.set_targets(target)
        self.assertEqual(link.get_state().pulses, target)
        self.assertTrue(serial.armed)
        link.disarm()
        self.assertFalse(serial.armed)
        serial.source = "LOCAL"
        link.handshake()
        with self.assertRaisesRegex(LinkError, "LOCAL"):
            link.arm(calibration.home_pulses)

    def test_live_timeout_is_bounded_and_does_not_retry_motion(self):
        serial = FakeSerial(self.calibration)
        serial.drop_ack = True
        link = SerialLink(serial, self.calibration, timeout=0.002)
        with self.assertRaisesRegex(LinkError, "No valid reply"):
            link.handshake()
        self.assertEqual(len(serial.commands), 1)

    def test_arm_resets_stale_slider_targets_and_slew_limits_applied_commands(self):
        model, data, arm, link, sync = self.make_sync()
        arm.set_joint_targets(yaw_deg=4)
        sync.arm()
        np.testing.assert_allclose(arm.target_degrees, self.calibration.home_degrees)
        arm.set_joint_targets(yaw_deg=5)
        sync.update(self.now)
        self.now += 0.1
        sync.update(self.now)
        self.assertEqual(sync.state.pulses[0], 1525)  # 250 us/s, not immediate1552
        self.assertEqual(arm.target_degrees[0], 5)
        self.assertTrue(sync.enabled)
        sync.close()
        self.assertFalse(link.firmware_armed)

    def test_watchdog_timeout_stays_disarmed_until_explicit_arm(self):
        _, _, arm, link, sync = self.make_sync()
        sync.arm()
        self.now = 0.6
        state = link.get_state()
        self.assertFalse(state.armed)
        self.assertEqual(state.reason, "TIMEOUT")
        with self.assertRaisesRegex(LinkError, "Not armed"):
            sync.update(self.now)
        self.assertFalse(sync.enabled)
        self.assertIsNotNone(sync.fault)
        with self.assertRaisesRegex(LinkError, "fault is latched"):
            sync.arm()

    def test_duplicate_arm_rejects_without_resetting_a_live_target(self):
        _, _, arm, _, sync = self.make_sync()
        sync.arm()
        arm.set_joint_targets(yaw_deg=3)
        before = arm.target.copy()
        with self.assertRaisesRegex(ValueError, "Already armed"):
            sync.arm()
        np.testing.assert_array_equal(arm.target, before)
        self.assertTrue(sync.enabled)

    def test_invalid_slider_target_fault_detaches_without_clamping(self):
        _, data, arm, link, sync = self.make_sync()
        sync.arm()
        data.ctrl[arm.actuator_ids[0]] = np.deg2rad(30)  # simulate a bad externalAPIwrite
        with self.assertRaisesRegex(LinkError, "outside calibrated range"):
            sync.update(self.now)
        self.assertFalse(link.firmware_armed)
        self.assertIsNotNone(sync.fault)

    def test_observer_updates_ctrl_not_physics_and_never_arms_or_disarms(self):
        _, data, arm, link, sync = self.make_sync("hardware-to-sim")
        link.arm(self.calibration.home_pulses)  # virtual LOCAL controller, outside sync
        link.set_targets(self.calibration.to_pulses((3, 92, -3, 15)))
        self.now += 0.2
        qpos = data.qpos.copy()
        sync.update(self.now)
        np.testing.assert_allclose(arm.target_degrees, self.calibration.to_degrees(sync.state.pulses))
        np.testing.assert_array_equal(data.qpos, qpos)
        with self.assertRaisesRegex(ValueError, "read-only"):
            sync.arm()
        with self.assertRaisesRegex(ValueError, "master"):
            handle_sync_command(sync, "yaw 1")
        sync.close()
        self.assertTrue(link.firmware_armed)

    def test_lost_arm_ack_triggers_disarm_and_fault_latch(self):
        _, _, _, link, sync = self.make_sync()
        original_arm = link.arm
        def lost_ack(pulses):
            original_arm(pulses)
            raise LinkError("Lost ARM acknowledgement")
        link.arm = lost_ack
        with self.assertRaisesRegex(LinkError, "Lost ARM"):
            sync.arm()
        self.assertFalse(link.firmware_armed)
        self.assertFalse(sync.enabled)
        self.assertIsNotNone(sync.fault)

    def test_estop_latches_even_if_reply_is_lost(self):
        _, _, _, link, sync = self.make_sync()
        sync.arm()
        sync.estop()
        self.assertFalse(link.firmware_armed)
        with self.assertRaisesRegex(LinkError, "fault is latched"):
            sync.arm()
        with self.assertRaisesRegex(LinkError, "ESTOP is latched"):
            link.arm(self.calibration.home_pulses)


if __name__ == "__main__":
    unittest.main()
