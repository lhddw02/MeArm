"""Measured servo mapping and commissioning-file checks; no hardware needed."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from scripts.calibrate_hardware import main
from scripts.hardware_calibration import Calibration


ROOT = Path(__file__).resolve().parents[1]


class HardwareCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.source = json.loads((ROOT / "config/hardware_calibration.example.json").read_text())
        self.calibration = Calibration.from_dict(self.source)

    def test_example_is_disarmed_and_home_maps_to_measured_anchors(self):
        self.assertFalse(self.calibration.calibrated)
        self.assertEqual(self.calibration.home_degrees, (0, 90, 0, 10))
        self.assertEqual(self.calibration.home_pulses, (1500, 1266, 1472, 1850))
        self.assertEqual(self.calibration.to_pulses(self.calibration.home_degrees),
                         self.calibration.home_pulses)
        self.assertEqual(self.calibration.to_degrees(self.calibration.home_pulses),
                         self.calibration.home_degrees)

    def test_both_mapping_directions_and_piecewise_non_linearity(self):
        self.source["joints"][0]["model_deg"] = [-5, 0, 2, 5]
        self.source["joints"][0]["pulse_us"] = [1400, 1500, 1560, 1590]
        calibration = Calibration.from_dict(self.source)
        pulses = calibration.to_pulses((3, 92.5, -2.5, 15))
        self.assertEqual(pulses, (1570, 1239, 1441, 1775))
        angles = calibration.to_degrees(pulses)
        self.assertAlmostEqual(angles[0], 3)
        self.assertAlmostEqual(angles[1], 90 + 27 / 11)
        self.assertAlmostEqual(angles[2], -2.5)
        self.assertAlmostEqual(angles[3], 15)
        for joint in calibration.joints:
            for degrees, pulse in zip(joint.model_deg, joint.pulse_us):
                self.assertEqual(joint.to_pulse(degrees), pulse)
                self.assertAlmostEqual(joint.to_degrees(pulse), degrees)

    def test_no_extrapolation_at_either_end_including_reversed_pulse_map(self):
        for joint in self.calibration.joints:
            with self.subTest(joint=joint.name):
                self.assertEqual(joint.to_pulse(joint.min_deg - 1e-12), joint.pulse_us[0])
                self.assertEqual(joint.to_pulse(joint.max_deg + 1e-12), joint.pulse_us[-1])
                self.assertAlmostEqual(joint.to_degrees(joint.max_us + 1e-12),
                                       joint.to_degrees(joint.max_us))
                for invalid in (joint.min_deg - 0.001, joint.max_deg + 0.001):
                    with self.assertRaisesRegex(ValueError, "outside calibrated range"):
                        joint.to_pulse(invalid)
                for invalid in (joint.min_us - 1, joint.max_us + 1):
                    with self.assertRaisesRegex(ValueError, "outside calibrated range"):
                        joint.to_degrees(invalid)

    def test_profile_id_is_order_independent_but_includes_safety_settings(self):
        reordered = dict(reversed(list(self.source.items())))
        reordered["joints"] = [dict(reversed(list(j.items()))) for j in self.source["joints"]]
        self.assertEqual(Calibration.from_dict(reordered).profile_id, self.calibration.profile_id)
        self.assertRegex(self.calibration.profile_id, r"^[0-9a-f]{8}$")
        for key, value in (("calibrated", True), ("watchdog_ms", 1000)):
            changed = deepcopy(self.source)
            changed[key] = value
            self.assertNotEqual(Calibration.from_dict(changed).profile_id,
                                self.calibration.profile_id)
        changed = deepcopy(self.source)
        changed["joints"][0]["max_us_per_second"] = 100
        self.assertNotEqual(Calibration.from_dict(changed).profile_id,
                            self.calibration.profile_id)

    def test_rejects_ambiguous_or_unsafe_configuration_without_mutation(self):
        variations = []
        for key, value in (("version", 2), ("version", True), ("calibrated", "true"),
                           ("baudrate", 9600), ("watchdog_ms", 199), ("watchdog_ms", 2001)):
            source = deepcopy(self.source)
            source[key] = value
            variations.append(source)
        for field, value in (("name", "shoulder"), ("pin", 0), ("pin", 10),
                             ("model_deg", [-5, -5, 5]), ("model_deg", [-46, 0, 5]),
                             ("model_deg", [-5, math.nan, 5]),
                             ("pulse_us", [1400, 1500, 1400]),
                             ("pulse_us", [899, 1500, 1600]),
                             ("pulse_us", [1400, 1500, 2101]),
                             ("pulse_us", [1400, 1500, 1600.5]),
                             ("pulse_us", [1400, 1500]),
                             ("home_deg", 6), ("home_deg", math.inf),
                             ("max_us_per_second", 0), ("max_us_per_second", 1001),
                             ("max_us_per_second", True)):
            source = deepcopy(self.source)
            source["joints"][0][field] = value
            variations.append(source)
        extra = deepcopy(self.source)
        extra["joint"] = []
        variations.append(extra)
        for source in variations:
            with self.subTest(source=source):
                before = repr(source)
                with self.assertRaises(ValueError):
                    Calibration.from_dict(source)
                self.assertEqual(repr(source), before)

    def test_vector_validation_is_atomic_and_rejects_nonfinite_values(self):
        valid = list(self.calibration.home_degrees)
        for invalid in (valid[:3], valid + [1], "0,90,0,10", None,
                        [0, 90, math.nan, 10], [0, 90, 0, True], [0, 90, 0, 21]):
            before = self.calibration.to_dict()
            with self.assertRaises(ValueError):
                self.calibration.to_pulses(invalid)
            self.assertEqual(self.calibration.to_dict(), before)
        self.assertEqual(self.calibration.to_pulses(iter(valid)), self.calibration.home_pulses)

    def test_generated_header_uses_measured_bounds_and_matching_profile(self):
        header = self.calibration.firmware_header()
        self.assertIn(f'#define MEARM_PROFILE_ID "{self.calibration.profile_id}"', header)
        self.assertIn("#define MEARM_CALIBRATED false", header)
        self.assertIn("#ifndef MEARM_SOURCE_LOCAL", header)
        self.assertIn("MEARM_PINS[4] = {11, 10, 9, 6}", header)
        self.assertIn("MEARM_MIN_US[4] = {1448, 1211, 1410, 1700}", header)
        self.assertIn("MEARM_MAX_US[4] = {1552, 1321, 1534, 2000}", header)
        self.assertIn("MEARM_HOME_US[4] = {1500, 1266, 1472, 1850}", header)
        self.assertIn("MEARM_WATCHDOG_MS = 500UL", header)

    def test_cli_init_check_export_and_overwrite_guards(self):
        (ROOT / "tmp").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "tmp") as directory:
            path = Path(directory) / "hardware.json"
            header = Path(directory) / "calibration.h"
            with redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main(["--init", str(path)]), 0)
                self.assertEqual(main(["--check", str(path)]), 0)
                self.assertEqual(main(["--export-firmware", str(path), "--output", str(header)]), 0)
            self.assertIn("UNCALIBRATED", output.getvalue())
            self.assertIn("calibrated=false", output.getvalue())
            self.assertEqual(header.read_text(), self.calibration.firmware_header())
            original = path.read_bytes()
            with redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as stopped:
                    main(["--init", str(path)])
                self.assertEqual(stopped.exception.code, 2)
                with self.assertRaises(SystemExit) as stopped:
                    main(["--export-firmware", str(path), "--output", str(path)])
                self.assertEqual(stopped.exception.code, 2)
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
