"""Headless control regression tests using the actual coupled MeArm model."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
import numpy as np

from arm_control import MeArmController, forward_kinematics, inverse_kinematics


class ArmControlTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/mearm.xml"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.arm = MeArmController(self.model, self.data)

    def test_neutral_position_and_units(self):
        np.testing.assert_allclose(forward_kinematics([0, 0, 0]), [148, 0, 135], atol=1e-10)
        np.testing.assert_allclose(self.arm.tcp_position_mm, [148, 0, 135], atol=1e-8)
        np.testing.assert_allclose(self.arm.move_to(148, 0, 135), [0, 0, 0, 0], atol=1e-10)
        np.testing.assert_allclose(self.arm.target_degrees, [0, 90, 0, 0], atol=1e-10)
        np.testing.assert_allclose(self.arm.limits_degrees[1], [45, 135], atol=1e-8)

    def test_random_valid_joint_targets_round_trip_through_ik(self):
        rng = np.random.default_rng(20261015)
        samples = rng.uniform([-44, -44, -44, 0], [44, 44, 44, 28], size=(150, 4))
        for degrees in samples:
            target_xyz = forward_kinematics(np.deg2rad(degrees))
            user_degrees = degrees.copy()
            user_degrees[1] += 90
            np.testing.assert_allclose(self.arm.set_joint_targets(*user_degrees), np.deg2rad(degrees))
            self.arm.set_joint_targets(0, 90, 0, degrees[3])
            solved = self.arm.move_to(*target_xyz)
            np.testing.assert_allclose(forward_kinematics(solved), target_xyz, atol=1e-8)
            np.testing.assert_allclose(solved, np.deg2rad(degrees), atol=1e-9)

    def test_ik_retains_current_valid_branch(self):
        # A straight pose at the edge has coincident branches; it must still
        # survive floating point noise at the stock servo boundaries.
        self.arm.set_joint_targets(45, 45, 45, 10)
        target = self.arm.forward_kinematics()
        np.testing.assert_allclose(self.arm.move_to(*target), np.deg2rad([45, -45, 45, 10]), atol=1e-7)
        self.assertEqual(len(inverse_kinematics(148, 0, 135)), 2)

    def test_commands_write_only_servo_targets(self):
        self.data.qpos[:] = np.linspace(-0.1, 0.1, self.model.nq)
        self.data.qvel[:] = np.linspace(-0.2, 0.2, self.model.nv)
        before_qpos, before_qvel = self.data.qpos.copy(), self.data.qvel.copy()
        self.arm.set_joint_targets(10, 75, 20, 12)
        np.testing.assert_array_equal(self.data.qpos, before_qpos)
        np.testing.assert_array_equal(self.data.qvel, before_qvel)
        np.testing.assert_allclose(self.arm.target, np.deg2rad([10, -15, 20, 12]))

    def test_partial_command_preserves_native_control_slider_changes(self):
        self.data.ctrl[self.arm.actuator_ids] = np.deg2rad([12, -18, 21, 11])
        self.arm.set_joint_targets(yaw_deg=-20)
        np.testing.assert_allclose(self.arm.target_degrees, [-20, 72, 21, 11])
        self.arm.move_to(148, 0, 135)
        np.testing.assert_allclose(self.arm.target_degrees, [0, 90, 0, 11], atol=1e-9)
        copy = self.arm.target
        copy[:] = 100
        np.testing.assert_allclose(self.arm.target_degrees, [0, 90, 0, 11], atol=1e-9)

    def test_named_actuators_work_when_xml_order_changes(self):
        xml = ET.parse(ROOT / "mujoco/mearm.xml").getroot()
        xml.find("compiler").set("meshdir", str(ROOT / "models/mearm_v1/meshes"))
        actuators = xml.find("actuator")
        actuators[:] = list(reversed(list(actuators)))
        model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
        data = mujoco.MjData(model)
        arm = MeArmController(model, data)
        arm.set_joint_targets(15, 80, 20, 5)
        for name, expected in zip(("base", "shoulder", "elbow", "gripper"), (15, -10, 20, 5)):
            self.assertAlmostEqual(data.ctrl[model.actuator(name).id], np.deg2rad(expected))

    def test_bad_joint_commands_are_atomic(self):
        self.arm.set_joint_targets(10, 75, 20, 12)
        baseline = self.data.ctrl.copy()
        for value in (float("nan"), float("inf"), -float("inf"), 180, "bad"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.arm.set_joint_targets(yaw_deg=0, shoulder_deg=value)
                np.testing.assert_array_equal(self.data.ctrl, baseline)
        for value in (-0.1, 29):
            with self.assertRaises(ValueError):
                self.arm.set_joint_targets(yaw_deg=0, grip_deg=value)
            np.testing.assert_array_equal(self.data.ctrl, baseline)

    def test_bad_cartesian_commands_are_atomic(self):
        self.arm.set_joint_targets(10, 75, 20, 12)
        baseline = self.data.ctrl.copy()
        cases = ((2000, 0, 135), (0, 148, 135), (68, 0, 55),
                 (float("nan"), 0, 135), (148, float("inf"), 135))
        for xyz in cases:
            with self.subTest(xyz=xyz):
                with self.assertRaises(ValueError):
                    self.arm.move_to(*xyz, grip_deg=0)
                np.testing.assert_array_equal(self.data.ctrl, baseline)
        for grip in (float("nan"), float("inf"), 40, -1):
            with self.assertRaises(ValueError):
                self.arm.move_to(148, 0, 135, grip_deg=grip)
            np.testing.assert_array_equal(self.data.ctrl, baseline)

    def test_position_command_settles_under_actual_physics(self):
        desired_xyz = forward_kinematics(np.deg2rad([20, -15, 10]))
        target = self.arm.move_to(*desired_xyz, grip_deg=15)
        for _ in range(round(5 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        error_mm = np.linalg.norm(self.arm.tcp_position_mm - desired_xyz)
        self.assertLess(error_mm, 2.0)
        joints = ("yaw_joint", "shoulder_joint", "elbow_joint", "left_finger_joint")
        actual = np.array([self.data.qpos[self.model.joint(name).qposadr[0]] for name in joints])
        self.assertLess(np.max(np.abs(actual - target)), 0.015)
        self.assertTrue(np.all(np.isfinite(self.data.qpos)))
        self.assertFalse(any(warning.number for warning in self.data.warning))

    def test_user_shoulder_angles_are_absolute_elevations(self):
        raw = np.deg2rad([20, -25, -20, 15])
        self.arm.set_joint_targets(20, 65, -20, 15)
        np.testing.assert_allclose(self.arm.target, raw)
        np.testing.assert_allclose(self.arm.target_degrees, [20, 65, -20, 15])
        np.testing.assert_allclose(self.arm.forward_kinematics(), forward_kinematics(raw))
        np.testing.assert_allclose(self.arm.forward_kinematics(20, 65, -20), forward_kinematics(raw))
        with self.assertRaisesRegex(ValueError, r"shoulder target 0.000 deg is outside \[45.000, 135.000\]"):
            self.arm.set_joint_targets(shoulder_deg=0)


if __name__ == "__main__":
    unittest.main()
