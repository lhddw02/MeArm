"""Headless checks for cube physics and manual/Cartesian scene control."""
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

from cube_scene import CUBES, build_scene_xml, load_scene
from run_cube_scene import CubeSceneController, cube_target_mm


class CubeSceneTests(unittest.TestCase):
    def setUp(self):
        self.model, self.data = load_scene()
        self.control = CubeSceneController(self.model, self.data)

    def step(self, seconds):
        for _ in range(round(seconds / self.model.opt.timestep)):
            self.control.update()
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def assert_physics_healthy(self):
        self.assertTrue(np.all(np.isfinite(self.data.qpos)))
        self.assertTrue(np.all(np.isfinite(self.data.qvel)))
        self.assertFalse(any(warning.number for warning in self.data.warning))

    def test_four_coloured_cubes_are_physical_twenty_mm_boxes(self):
        self.assertEqual({cube["name"] for cube in CUBES},
                         {"red", "green", "blue", "yellow"})
        expected = {
            "red": [130, -45, 90], "green": [180, 0, 90],
            "blue": [130, 45, 90], "yellow": [180, 45, 90],
        }
        colours = []
        for name, position_mm in expected.items():
            with self.subTest(cube=name):
                body = self.model.body("cube_" + name)
                geom = self.model.geom("cube_" + name + "_geom")
                joint = self.model.joint("cube_" + name + "_joint")
                self.assertEqual(self.model.geom_type[geom.id], mujoco.mjtGeom.mjGEOM_BOX)
                np.testing.assert_allclose(self.model.geom_size[geom.id], [.01, .01, .01])
                self.assertEqual(self.model.jnt_type[joint.id], mujoco.mjtJoint.mjJNT_FREE)
                self.assertAlmostEqual(self.model.body_mass[body.id], .008)
                self.assertTrue(np.all(self.model.body_inertia[body.id] > 0))
                np.testing.assert_allclose(self.data.xpos[body.id] * 1000, position_mm)
                self.assertTrue(self.model.geom_contype[geom.id])
                self.assertTrue(self.model.geom_conaffinity[geom.id])
                colours.append(tuple(self.model.geom_rgba[geom.id]))
        self.assertEqual(len(set(colours)), 4)

    def test_platform_supports_cubes_after_settling(self):
        self.step(2)
        platform_id = self.model.geom("platform_top").id
        contacts = {frozenset((contact.geom1, contact.geom2))
                    for contact in self.data.contact[:self.data.ncon]}
        for cube in CUBES:
            with self.subTest(cube=cube["name"]):
                body_id = self.model.body("cube_" + cube["name"]).id
                geom_id = self.model.geom("cube_" + cube["name"] + "_geom").id
                np.testing.assert_allclose(self.data.xpos[body_id] * 1000,
                                           cube["position_mm"], atol=.1)
                self.assertIn(frozenset((platform_id, geom_id)), contacts)
        self.assert_physics_healthy()

    def test_scene_generation_does_not_change_source_arm(self):
        source_path = ROOT / "mujoco/mearm.xml"
        before = source_path.read_bytes()
        scene = ET.fromstring(build_scene_xml([170, 10, 125]))
        self.assertEqual(source_path.read_bytes(), before)
        original = ET.fromstring(before)
        for path in ("worldbody/body[@name='base']", "actuator", "equality"):
            self.assertEqual(
                ET.canonicalize(ET.tostring(scene.find(path), encoding="unicode"), strip_text=True),
                ET.canonicalize(ET.tostring(original.find(path), encoding="unicode"), strip_text=True))
        target = np.fromstring(scene.find("worldbody/site[@name='reach_target']").get("pos"),
                               sep=" ")
        np.testing.assert_allclose(target, [.170, .010, .125])

    def test_manual_mode_preserves_native_joint_slider_commands(self):
        self.assertEqual(self.control.mode, "manual")
        self.data.ctrl[self.control.arm.actuator_ids] = np.deg2rad([12, -15, 10, 15])
        expected = self.data.ctrl.copy()
        self.step(.25)
        np.testing.assert_array_equal(self.data.ctrl, expected)
        np.testing.assert_allclose(self.control.arm.target_degrees, [12, 75, 10, 15])
        self.assert_physics_healthy()

    def test_automatic_reach_moves_physics_to_target(self):
        goal = np.array([160, 0, 120])
        self.control.set_auto(goal, move_seconds=2)
        self.assertEqual(self.control.mode, "auto")
        np.testing.assert_allclose(self.control.target_mm, goal)
        self.step(5)
        self.assertTrue(self.control.finished)
        self.assertLess(self.control.error_mm, 3)
        self.assertLess(np.linalg.norm(self.control.arm.tcp_position_mm - goal), 3)
        np.testing.assert_allclose(self.model.site_pos[self.model.site("reach_target").id],
                                   goal / 1000)
        self.assert_physics_healthy()
        held_command = self.data.ctrl.copy()
        self.data.ctrl[:] = 0
        self.control.update()
        np.testing.assert_allclose(self.data.ctrl, held_command)

    def test_automatic_control_resolves_actuators_by_name(self):
        scene = ET.fromstring(build_scene_xml())
        actuators = scene.find("actuator")
        actuators[:] = list(reversed(list(actuators)))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.control = CubeSceneController(self.model, self.data)
        self.control.set_auto([160, 0, 120], move_seconds=1, grip_deg=15)
        self.step(5)
        self.assertLess(self.control.error_mm, 3)
        self.assertAlmostEqual(self.data.ctrl[self.model.actuator("gripper").id],
                               np.deg2rad(15))
        self.assert_physics_healthy()

    def test_each_cube_has_a_reachable_hover_target(self):
        for cube in CUBES:
            with self.subTest(cube=cube["name"]):
                self.model, self.data = load_scene()
                self.control = CubeSceneController(self.model, self.data)
                target = cube_target_mm(self.model, self.data, cube["name"])
                np.testing.assert_allclose(target, np.array(cube["position_mm"]) + [0, 0, 30])
                self.control.set_auto(target, move_seconds=2, grip_deg=28)
                self.step(5)
                self.assertLess(self.control.error_mm, 3)
                self.assert_physics_healthy()

    def test_cube_target_follows_current_simulated_position(self):
        joint = self.model.joint("cube_red_joint")
        self.data.qpos[joint.qposadr[0]:joint.qposadr[0] + 3] = [.14, -.04, .10]
        mujoco.mj_forward(self.model, self.data)
        np.testing.assert_allclose(cube_target_mm(self.model, self.data, "red"),
                                   [140, -40, 130])

    def test_automatic_command_does_not_teleport_arm_or_free_cubes(self):
        # Give each cube a distinct non-default state to detect a scene reset.
        for index, cube in enumerate(CUBES):
            address = self.model.joint("cube_" + cube["name"] + "_joint").qposadr[0]
            self.data.qpos[address] += (index + 1) * .001
        self.data.qvel[:] = np.linspace(-.01, .01, self.model.nv)
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        ctrl = self.data.ctrl.copy()
        self.control.set_auto([160, 0, 120], move_seconds=2, grip_deg=15)
        self.control.update()
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)
        np.testing.assert_allclose(self.data.ctrl, ctrl, atol=1e-12)
        self.step(.5)
        self.assertGreater(np.linalg.norm(self.data.ctrl - ctrl), .001)

    def test_invalid_auto_requests_leave_running_motion_unchanged(self):
        self.control.set_auto([160, 0, 120], move_seconds=2)
        self.step(.3)
        baseline = self.data.ctrl.copy()
        target = np.array(self.control.target_mm, copy=True)
        marker = self.model.site_pos[self.model.site("reach_target").id].copy()
        invalid = [
            ([2000, 0, 120], 2, None), ([0, 160, 120], 2, None),
            ([float("nan"), 0, 120], 2, None), ([160, 0], 2, None),
            ([160, 0, 120, 0], 2, None), ("bad", 2, None),
            ([160, 0, 120], 0, None), ([160, 0, 120], -1, None),
            ([160, 0, 120], float("nan"), None),
            ([160, 0, 120], float("inf"), None),
            ([160, 0, 120], 2, 40), ([160, 0, 120], 2, float("nan")),
        ]
        for xyz, duration, grip in invalid:
            with self.subTest(target=xyz, duration=duration, grip=grip):
                with self.assertRaises(ValueError):
                    self.control.set_auto(xyz, move_seconds=duration, grip_deg=grip)
                self.assertEqual(self.control.mode, "auto")
                np.testing.assert_array_equal(self.data.ctrl, baseline)
                np.testing.assert_array_equal(self.control.target_mm, target)
                np.testing.assert_array_equal(
                    self.model.site_pos[self.model.site("reach_target").id], marker)

    def test_switching_to_manual_releases_automatic_control(self):
        self.control.set_auto([160, 0, 120], move_seconds=2)
        self.step(.25)
        self.control.set_manual()
        self.assertEqual(self.control.mode, "manual")
        self.control.arm.set_joint_targets(10, 90, 10, 15)
        expected = self.data.ctrl.copy()
        self.step(.25)
        np.testing.assert_array_equal(self.data.ctrl, expected)
        self.assert_physics_healthy()


if __name__ == "__main__":
    unittest.main()
