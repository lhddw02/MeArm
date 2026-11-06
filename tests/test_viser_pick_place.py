"""Run Viser's application loop with real MuJoCo physics and lightweight UI handles."""
import unittest

import numpy as np

from scripts.cube_scene import CUBES
from scripts.run_viser import ViserSimulation
from tests.test_scene_environment import FakeServer


class FakeControl:
    def __init__(self, **kwargs):
        self.value = kwargs.get("initial_value")
        self.disabled = kwargs.get("disabled", False)
        self.click_callbacks = []
        self.update_callbacks = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def on_click(self, callback):
        self.click_callbacks.append(callback)
        return callback

    def on_update(self, callback):
        self.update_callbacks.append(callback)
        return callback


class FakeGui:
    def configure_theme(self, **kwargs):
        pass

    def set_panel_label(self, label):
        pass

    def __getattr__(self, name):
        if name.startswith("add_"):
            return lambda *args, **kwargs: FakeControl(**kwargs)
        raise AttributeError(name)


def make_app(scene="empty", task="cubes"):
    server = FakeServer()
    server.gui = FakeGui()
    return ViserSimulation(server, scene, task=task)


class ViserPickPlaceTests(unittest.TestCase):
    def test_default_remains_empty_without_cubes(self):
        app = make_app(task="arm")
        self.assertIsNone(app.pick_task)
        self.assertFalse(hasattr(app, "pick_button"))
        self.assertEqual(app.environment.name, "Empty")
        with self.assertRaisesRegex(ValueError, "--task cubes"):
            app.start_pick_place("red", [180, -45, 90])

    def test_each_color_is_physically_picked_and_rendered_at_destination(self):
        for cube in CUBES:
            with self.subTest(color=cube["name"]):
                app = make_app()
                color = cube["name"]
                initial_qpos = app.data.qpos.copy()
                initial_height = app.data.xpos[app.model.body("cube_" + color).id, 2]
                app.start_pick_place(color, [180, -45, 90])
                np.testing.assert_array_equal(app.data.qpos, initial_qpos)
                self.assertTrue(app.sliders[0].disabled)
                self.assertFalse(app.cancel_pick_button.disabled)
                app.advance(0)
                marker = next(node for sid, node in app.renderer.site_nodes
                              if app.model.site(sid).name == "place_target")
                self.assertTrue(marker.visible)
                np.testing.assert_allclose(marker.position, [.18, -.045, .0805])
                max_height = initial_height
                saw_grasp_contact = False
                for _ in range(260):
                    app.advance(.1)
                    max_height = max(max_height, app.pick_task.cube_position_mm[2] / 1000)
                    saw_grasp_contact |= app.pick_task.bilateral_contacts()
                    if not app.pick_task.active:
                        break
                result = app.report()["pick_place"]
                self.assertEqual(result["status"], "complete", result)
                self.assertTrue(result["released"])
                self.assertLess(result["placement_error_mm"], 3)
                self.assertGreater(max_height - initial_height, .015)
                self.assertTrue(saw_grasp_contact)
                self.assertFalse(app.sliders[0].disabled)
                self.assertIn("complete", app.pick_status.value)
                geom_id = app.model.geom("cube_" + color + "_geom").id
                cube_node = next(node for gid, node in app.renderer.nodes if gid == geom_id)
                np.testing.assert_allclose(cube_node.position, app.data.geom_xpos[geom_id], atol=1e-12)
                np.testing.assert_allclose(cube_node.position * 1000, [180, -45, 90], atol=3)
                self.assertFalse(any(w.number for w in app.data.warning))
                app.commands.put(("joint", (0, 5.)))
                app.advance(.2)
                self.assertAlmostEqual(app.arm.target_degrees[0], 5.)
                self.assertEqual(app.motion.mode, "manual")
                self.assertEqual(app.pick_task.status, "idle")
                self.assertFalse(marker.visible)

    def test_pause_freezes_task_and_cancel_restores_manual_control(self):
        app = make_app("workshop")
        app.start_pick_place("red", [180, -45, 90])
        app.advance(7.6)
        app.commands.put(("pause", True))
        app.advance(1.)
        before = app.data.qpos.copy()
        sim_time = app.data.time
        phase = app.pick_task.phase
        app.advance(1.)
        self.assertEqual(app.data.time, sim_time)
        self.assertEqual(app.pick_task.phase, phase)
        np.testing.assert_array_equal(app.data.qpos, before)
        app.commands.put(("joint", (0, 20.)))
        app.advance(0)
        self.assertIn("Cancel", app.status.value)
        self.assertTrue(app.pick_task.active)
        renderer = app.renderer
        with self.assertRaisesRegex(ValueError, "Cancel"):
            app.load_scene("classroom")
        self.assertIs(app.renderer, renderer)
        app.commands.put(("cancel_pick", None))
        app.advance(0)
        self.assertEqual(app.pick_task.status, "cancelled")
        self.assertFalse(app.sliders[0].disabled)
        app.commands.put(("joint", (0, 12.)))
        app.advance(0)
        self.assertAlmostEqual(app.arm.target_degrees[0], 12.)
        self.assertEqual(app.motion.mode, "manual")

    def test_invalid_pick_is_atomic_and_reset_restores_cubes(self):
        app = make_app()
        app.demo = True
        before = app.data.qpos.copy()
        ctrl = app.data.ctrl.copy()
        with self.assertRaisesRegex(ValueError, "overlaps"):
            app.start_pick_place("red", [180, 0, 90])  # Green cube occupies it.
        self.assertTrue(app.demo)
        self.assertEqual(app.pick_task.status, "idle")
        np.testing.assert_array_equal(app.data.qpos, before)
        np.testing.assert_array_equal(app.data.ctrl, ctrl)
        app.start_pick_place("blue", [180, -45, 90])
        self.assertFalse(app.demo)
        app.advance(.8)
        app.commands.put(("reset", None))
        app.advance(0)
        self.assertEqual(app.pick_task.status, "idle")
        self.assertFalse(app.pick_button.disabled)
        self.assertFalse(app.model.site_rgba[app.model.site('place_target').id, 3])
        for cube in CUBES:
            actual = app.data.xpos[app.model.body('cube_' + cube['name']).id] * 1000
            np.testing.assert_allclose(actual, cube['position_mm'], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
