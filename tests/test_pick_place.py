"""End-to-end physical grasping and transactional pick-and-place checks."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
import numpy as np

from cube_scene import CUBES, load_scene
from run_cube_scene import CubeSceneController
from pick_place import PickPlaceTask


class PickPlaceTests(unittest.TestCase):
    def make_task(self):
        model, data = load_scene(grasp_ready=True)
        motion = CubeSceneController(model, data)
        return model, data, motion, PickPlaceTask(motion)

    def cube_positions(self, model, data):
        return {cube['name']: 1000 * data.xpos[int(model.body('cube_' + cube['name']).id)].copy()
                for cube in CUBES}

    def assert_no_active_weld(self, model, data):
        welds = model.eq_type == int(mujoco.mjtEq.mjEQ_WELD)
        self.assertFalse(np.any(data.eq_active[welds]))

    def test_each_color_is_physically_lifted_carried_released_and_settled(self):
        for cube in CUBES:
            color = cube['name']
            with self.subTest(color=color):
                model, data, motion, task = self.make_task()
                initial = self.cube_positions(model, data)
                free_indices = []
                for item in CUBES:
                    joint_id = int(model.joint('cube_' + item['name'] + '_joint').id)
                    address = int(model.jnt_qposadr[joint_id])
                    free_indices.extend(range(address, address + 7))
                original = data.qpos[free_indices].copy()
                task.start(color, [180, -45, 90])
                self.assertFalse(task.report()['released'])
                np.testing.assert_array_equal(data.qpos[free_indices], original)
                max_height = initial[color][2]
                contacts_in_close = contacts_in_lift = False
                settled_states = []
                for _ in range(30000):
                    # Observing free-joint state around the controller call
                    # catches even a one-step teleport or hidden reset.
                    before = data.qpos[free_indices].copy()
                    task.update()
                    self.assertTrue(np.array_equal(data.qpos[free_indices], before),
                                    'Task must never write cube free-joint positions.')
                    if task.phase == 'close' and task.bilateral_contacts():
                        contacts_in_close = True
                        self.assertFalse(task.report()['released'])
                    if task.phase == 'lift' and task.bilateral_contacts():
                        contacts_in_lift = True
                    if task.phase == 'settle':
                        settled_states.append((float(data.time), task.cube_position_mm.copy()))
                    mujoco.mj_step(model, data)
                    max_height = max(max_height, task.cube_position_mm[2])
                    if not task.active:
                        break
                result = task.report()
                self.assertEqual(task.status, 'complete', result)
                self.assertTrue(result['released'])
                self.assertLess(result['placement_error_mm'], 3.0)
                self.assertGreater(max_height - initial[color][2], 15.0)
                self.assertTrue(contacts_in_close)
                self.assertTrue(contacts_in_lift)
                self.assert_no_active_weld(model, data)
                phases = [entry['phase'] for entry in task.history]
                self.assertEqual(phases, ['open', 'approach', 'descend', 'close', 'lift',
                                          'transfer', 'lower', 'release', 'retreat',
                                          'settle', 'complete'])
                self.assertGreaterEqual(task.history[-1]['time'] - task.history[-2]['time'], .5)
                self.assertGreater(len(settled_states), 490)
                for _, center in settled_states[-490:]:
                    self.assertLess(np.linalg.norm(center - [180, -45, 90]), 3.0)
                linear, angular = task._velocity(task._joint_id)
                self.assertLessEqual(np.linalg.norm(linear), .01)
                self.assertLessEqual(np.linalg.norm(angular), .2)
                for other, center in self.cube_positions(model, data).items():
                    if other != color:
                        self.assertLess(np.linalg.norm(center - initial[other]), .5,
                                        f'{color} job disturbed {other}')
                self.assertTrue(np.all(np.isfinite(data.qpos)))
                self.assertTrue(np.all(np.isfinite(data.qvel)))
                self.assertFalse(any(warning.number for warning in data.warning))

    def test_invalid_requests_preserve_active_job_and_commands_atomically(self):
        model, data, motion, task = self.make_task()
        task.start('red', [180, -45, 90])
        ctrl = data.ctrl.copy()
        qpos, qvel = data.qpos.copy(), data.qvel.copy()
        history = deepcopy(task.history)
        target, start, goal = motion.target_mm.copy(), motion._start.copy(), motion._goal.copy()
        marker_positions, marker_colors = model.site_pos.copy(), model.site_rgba.copy()
        for color, destination in [
                ('purple', [180, -45, 90]),
                ('red', [float('nan'), -45, 90]),
                ('red', [180, -45]),
                ('red', [180, -45, 110]),
                ('red', [240, -45, 90]),
                ('red', [180, 0, 90]),
                ('red', [200, -45, 90])]:
            with self.subTest(color=color, destination=destination):
                with self.assertRaises(ValueError):
                    task.start(color, destination)
                np.testing.assert_array_equal(data.ctrl, ctrl)
                np.testing.assert_array_equal(data.qpos, qpos)
                np.testing.assert_array_equal(data.qvel, qvel)
                np.testing.assert_array_equal(motion.target_mm, target)
                np.testing.assert_array_equal(motion._start, start)
                np.testing.assert_array_equal(motion._goal, goal)
                np.testing.assert_array_equal(model.site_pos, marker_positions)
                np.testing.assert_array_equal(model.site_rgba, marker_colors)
                self.assertEqual(task.history, history)
                self.assertTrue(task.active)
                self.assertEqual(task.status, 'running')
                self.assertEqual(task.phase, 'open')
                self.assertEqual(task.color, 'red')
                self.assertEqual(motion.mode, 'auto')

    def test_cancel_returns_joint_control_to_manual_without_moving_cubes(self):
        model, data, motion, task = self.make_task()
        task.start('red', [180, -45, 90])
        for _ in range(200):
            task.update()
            mujoco.mj_step(model, data)
        qpos, qvel = data.qpos.copy(), data.qvel.copy()
        task.cancel()
        self.assertEqual(motion.mode, 'manual')
        self.assertEqual(task.status, 'cancelled')
        self.assertFalse(task.active)
        self.assertFalse(task.report()['released'])
        self.assert_no_active_weld(model, data)
        np.testing.assert_array_equal(data.qpos, qpos)
        np.testing.assert_array_equal(data.qvel, qvel)
        motion.arm.set_joint_targets(yaw_deg=20)
        ctrl = data.ctrl.copy()
        task.update()
        np.testing.assert_array_equal(data.ctrl, ctrl)

    def test_missing_finger_contacts_fail_instead_of_claiming_success(self):
        model, data, motion, task = self.make_task()
        for index in range(model.ngeom):
            name = model.geom(index).name or ''
            if 'finger_collision' in name:
                model.geom_contype[index] = 0
                model.geom_conaffinity[index] = 0
        task.start('red', [180, -45, 90])
        for _ in range(20000):
            task.update()
            mujoco.mj_step(model, data)
            if not task.active:
                break
        self.assertEqual(task.status, 'failed')
        self.assertIn('missed', task.error.lower())
        self.assertEqual(motion.mode, 'manual')
        self.assertNotIn('complete', [entry['phase'] for entry in task.history])
        self.assertLess(task.cube_position_mm[2], 91)
        self.assert_no_active_weld(model, data)

    def test_reset_clears_old_result_without_changing_new_reach_or_dynamic_state(self):
        model, data, motion, task = self.make_task()
        task.start('red', [180, -45, 90])
        with self.assertRaises(ValueError):
            task.reset()
        self.assertTrue(task.active)
        self.assertEqual(task.color, 'red')
        task.cancel()
        motion.set_auto([160, 0, 120], 2.0)
        qpos, qvel, ctrl = data.qpos.copy(), data.qvel.copy(), data.ctrl.copy()
        target, start, goal = motion.target_mm.copy(), motion._start.copy(), motion._goal.copy()
        task.reset()
        self.assertEqual(task.status, 'idle')
        self.assertEqual(task.phase, 'idle')
        self.assertIsNone(task.color)
        self.assertIsNone(task.destination_mm)
        self.assertEqual(task.history, [])
        self.assertFalse(task.active)
        self.assertEqual(motion.mode, 'auto')
        np.testing.assert_array_equal(data.qpos, qpos)
        np.testing.assert_array_equal(data.qvel, qvel)
        np.testing.assert_array_equal(data.ctrl, ctrl)
        np.testing.assert_array_equal(motion.target_mm, target)
        np.testing.assert_array_equal(motion._start, start)
        np.testing.assert_array_equal(motion._goal, goal)
        marker = int(model.site('place_target').id)
        self.assertEqual(model.site_rgba[marker, 3], 0)

    def test_actual_shifted_source_position_is_used_without_resetting_it(self):
        model, data, motion, task = self.make_task()
        joint = int(model.joint('cube_red_joint').id)
        address = int(model.jnt_qposadr[joint])
        # This test fixture moves a cube before the task exists; the task must
        # read that actual position instead of resetting it to scene metadata.
        data.qpos[address:address + 3] = [.14, -.05, .09]
        mujoco.mj_forward(model, data)
        before = data.qpos.copy()
        task.start('red', [180, -45, 90])
        np.testing.assert_array_equal(data.qpos, before)
        descend = next(item for item in task._plan if item[0] == 'descend')
        np.testing.assert_allclose(descend[1], [140, -50, 104], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
