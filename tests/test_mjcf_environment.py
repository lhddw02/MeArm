"""Native reconstructed scenes retain physics, textures and their compiled frames."""
import json
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from PIL import Image

from scripts.arm_control import MeArmController
from scripts.run_viser import PhysicsSceneRenderer, ViserSimulation
from scripts.scene_environment import ROOT, load_environment, transform_matrix
from tests.test_scene_environment import FakeServer
from tests.test_viser_pick_place import FakeGui


class NativeMjcfEnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)
        assets = self.folder / "native assets"
        assets.mkdir()
        # Deliberately offset, asymmetric vertices force MuJoCo to recenter and
        # rotate the mesh. Separate texture indices exercise UV seam handling.
        (assets / "painted.obj").write_text(
            "v .2 .1 .05\nv .5 .1 .05\nv .25 .35 .05\nv .3 .15 .4\n"
            "vt 0 0\nvt 1 0\nvt 0 1\n"
            "vt 1 1\nvt .1 .2\nvt .3 .4\n"
            "vt .5 .6\nvt .7 .8\nvt .9 1\n"
            "vt .2 .1\nvt .4 .3\nvt .6 .5\n"
            "f 1/1 3/2 2/3\nf 1/4 2/5 4/6\n"
            "f 2/7 3/8 4/9\nf 3/10 1/11 4/12\n")
        self.pixels = np.array([[[255, 0, 0], [0, 255, 0]],
                                [[0, 0, 255], [255, 255, 0]]], dtype=np.uint8)
        Image.fromarray(self.pixels).save(assets / "painted.png")
        self.xml = self.folder / "native room.xml"
        self.xml.write_text('''<mujoco model="native_room">
  <compiler angle="radian" fusestatic="false"/>
  <option timestep=".007" gravity="0 0 -1"/>
  <asset>
    <mesh name="painted_mesh" file="native assets/painted.obj"/>
    <texture name="painted_texture" type="2d" file="native assets/painted.png"/>
    <material name="painted_material" texture="painted_texture" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 .1" pos="0 0 -.2"/>
    <body name="furniture" pos="3.1 .6 1.2" euler=".2 -.3 .4">
      <body name="shelf" pos=".1 -.2 .05" euler="-.1 .15 -.25">
        <geom name="painted" type="mesh" mesh="painted_mesh" material="painted_material"
              pos=".07 .08 .09" euler=".1 .2 .3" group="2" contype="0" conaffinity="0"/>
        <geom name="collision" type="box" size=".2 .1 .1" pos=".1 .2 0"
              contype="2" conaffinity="4" condim="4" friction=".7 .04 .002"
              group="3" rgba="0 0 0 0"/>
      </body>
    </body>
    <body name="dynamic_ball" pos="3.2 .6 1.8">
      <freejoint name="ball_free"/>
      <geom name="ball" type="sphere" size=".02" density="500"/>
    </body>
  </worldbody>
</mujoco>''')
        self.descriptor = self.folder / "native descriptor.json"
        self.descriptor.write_text(json.dumps({"version": 1, "name": "Native room",
                                               "mjcf": self.xml.name, "objects": []}))
        self.source = mujoco.MjModel.from_xml_path(str(self.xml))
        self.source_data = mujoco.MjData(self.source)
        mujoco.mj_forward(self.source, self.source_data)

    def test_standalone_xml_and_descriptor_preserve_native_physics(self):
        for reference in (self.xml, self.descriptor):
            with self.subTest(reference=reference):
                model, data, _ = load_environment(reference, include_robot=False)
                self.assertEqual((model.ngeom, model.nq, model.nu),
                                 (self.source.ngeom, self.source.nq, self.source.nu))
                self.assertEqual(model.ntex, 1)
                self.assertAlmostEqual(model.opt.timestep, .007)
                np.testing.assert_array_equal(model.opt.gravity, [0, 0, -1])
                np.testing.assert_allclose(data.geom_xpos, self.source_data.geom_xpos, atol=1e-12)
                np.testing.assert_allclose(data.geom_xmat, self.source_data.geom_xmat, atol=1e-12)
                gid = model.geom("collision").id
                self.assertEqual(model.geom_contype[gid], 2)
                self.assertEqual(model.geom_conaffinity[gid], 4)
                self.assertEqual(model.geom_condim[gid], 4)
                np.testing.assert_allclose(model.geom_friction[gid], [.7, .04, .002])
                self.assertEqual(sum(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE), 1)

    def test_combined_native_scene_preserves_arm_controls_and_source_collision(self):
        model, data, _ = load_environment(self.descriptor)
        stock = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/mearm.xml"))
        self.assertEqual(model.nu, 4)
        np.testing.assert_array_equal(model.actuator_ctrlrange, stock.actuator_ctrlrange)
        self.assertEqual(model.opt.timestep, stock.opt.timestep)
        np.testing.assert_array_equal(model.opt.gravity, stock.opt.gravity)
        controller = MeArmController(model, data)
        controller.set_joint_targets(12, 70, -10, 15)
        np.testing.assert_allclose(controller.target_degrees, [12, 70, -10, 15])
        self.assertEqual(sum(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE), 1)
        for name in ("painted", "collision", "ball", "ground"):
            source_id = self.source.geom(name).id
            imported_id = model.geom("environment_" + name).id
            np.testing.assert_allclose(data.geom_xpos[imported_id],
                                       self.source_data.geom_xpos[source_id], atol=1e-10)
            np.testing.assert_allclose(data.geom_xmat[imported_id],
                                       self.source_data.geom_xmat[source_id], atol=1e-10)
            self.assertEqual(model.geom_contype[imported_id], self.source.geom_contype[source_id])
            self.assertEqual(model.geom_conaffinity[imported_id], self.source.geom_conaffinity[source_id])

    def test_native_rigid_transform_preserves_all_compiled_world_frames(self):
        position, rpy = [.4, -.3, .2], [15, -20, 30]
        outer = transform_matrix(position, rpy)
        for include_robot in (False, True):
            with self.subTest(include_robot=include_robot):
                model, data, _ = load_environment(self.xml, include_robot=include_robot,
                                                  position=position, rpy=rpy)
                for name in ("painted", "collision", "ball", "ground"):
                    sid = self.source.geom(name).id
                    gid = model.geom(("environment_" if include_robot else "") + name).id
                    expected = outer[:3, :3] @ self.source_data.geom_xpos[sid] + outer[:3, 3]
                    np.testing.assert_allclose(data.geom_xpos[gid], expected, atol=1e-10)
                    np.testing.assert_allclose(data.geom_xmat[gid].reshape(3, 3),
                                               outer[:3, :3] @ self.source_data.geom_xmat[sid].reshape(3, 3),
                                               atol=1e-10)

    def test_native_scaling_and_invalid_descriptors_fail_explicitly(self):
        for scale in (.001, [1, 2, 1], 0, float("nan")):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                load_environment(self.xml, include_robot=False, scale=scale)
        cases = [
            {"version": 1, "mjcf": "missing.xml", "objects": []},
            {"version": 1, "mjcf": 12, "objects": []},
            {"version": 1, "mjcf": self.xml.name,
             "objects": [{"name": "extra", "type": "box", "size_m": [1, 1, 1]}]},
        ]
        for definition in cases:
            with self.subTest(definition=definition), self.assertRaises(ValueError):
                self.descriptor.write_text(json.dumps(definition))
                load_environment(self.descriptor, include_robot=False)
        broken = self.folder / "broken.xml"
        broken.write_text('<mujoco><worldbody><geom type="bogus"/></worldbody></mujoco>')
        with self.assertRaises(ValueError):
            load_environment(broken, include_robot=False)

    def test_viser_texture_uv_seams_and_world_vertices_match_compiled_mujoco(self):
        model, data, environment = load_environment(self.descriptor, include_robot=False)
        server = FakeServer()
        renderer = PhysicsSceneRenderer(server, model, data, environment)
        gid = model.geom("painted").id
        node = next(node for index, node in renderer.nodes if index == gid)
        mesh = node.kwargs["mesh"]
        self.assertEqual(mesh.visual.kind, "texture")
        self.assertGreater(len(mesh.vertices), 4)  # Original vertex indices share UV seams.
        mid = int(model.geom_dataid[gid])
        va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
        fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
        ta = int(model.mesh_texcoordadr[mid])
        expected_faces = model.mesh_face[fa:fa + fn]
        expected_vertices = model.mesh_vert[va:va + vn][expected_faces]
        actual_vertices = mesh.vertices[mesh.faces]
        np.testing.assert_allclose(actual_vertices, expected_vertices, atol=1e-12)
        compiled_uv = model.mesh_texcoord[ta + model.mesh_facetexcoord[fa:fa + fn]]
        # MuJoCo compiles OBJ V into top-down image coordinates; Trimesh's
        # TextureVisuals needs authored OBJ UVs (its GLB exporter flips V).
        expected_uv = compiled_uv.copy()
        expected_uv[..., 1] = 1 - expected_uv[..., 1]
        np.testing.assert_allclose(mesh.visual.uv[mesh.faces], expected_uv, atol=1e-12)
        authored_corner = np.all(np.isclose(compiled_uv, [.1, .8], atol=1e-7), axis=-1)
        self.assertEqual(int(authored_corner.sum()), 1)
        np.testing.assert_allclose(mesh.visual.uv[mesh.faces][authored_corner], [[.1, .2]], atol=1e-7)
        image = getattr(mesh.visual.material, "image", None)
        if image is None:
            image = mesh.visual.material.baseColorTexture
        self.assertEqual(image.size, (2, 2))
        np.testing.assert_array_equal(np.asarray(image.convert("RGB")), self.pixels)
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, node.wxyz)
        actual_world = actual_vertices @ rotation.reshape(3, 3).T + node.position
        expected_world = expected_vertices @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
        np.testing.assert_allclose(actual_world, expected_world, atol=1e-12)
        hidden = model.geom("collision").id
        self.assertNotIn(hidden, [index for index, _ in renderer.nodes])

    def test_native_camera_fits_far_geometry_and_ignores_infinite_plane(self):
        model, data, environment = load_environment(self.xml, include_robot=False)
        renderer = PhysicsSceneRenderer(FakeServer(), model, data, environment)
        view = renderer.camera_view()
        visible_vertices = []
        for gid, _ in renderer.nodes:
            if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_PLANE:
                continue
            mesh = renderer.geometry_mesh(model, gid)
            visible_vertices.extend(mesh.vertices @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid])
        vertices = np.asarray(visible_vertices)
        expected_center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
        np.testing.assert_allclose(view["look_at"], expected_center, atol=1e-7)
        self.assertGreater(view["look_at"][0], 2.5)
        # The 100 m display floor must not make the useful scene microscopic.
        self.assertLess(np.linalg.norm(view["position"] - view["look_at"]), 10)
        self.assertGreater(view["near"], 0)
        self.assertGreater(view["far"], np.max(np.linalg.norm(vertices - view["position"], axis=1)))

    def test_no_robot_application_advances_pauses_resets_and_switches_scene(self):
        server = FakeServer()
        server.gui = FakeGui()
        app = ViserSimulation(server, self.descriptor, include_robot=False)
        self.assertIsNone(app.arm)
        self.assertIsNone(app.motion)
        self.assertIsNone(app.pick_task)
        self.assertFalse(app.report()["robot_included"])
        initial = app.data.qpos.copy()
        app.advance(.14)
        self.assertGreater(app.data.time, 0)
        self.assertLess(app.data.qpos[2], initial[2])
        app.commands.put(("pause", True))
        app.advance(.14)
        paused_time = app.data.time
        app.advance(.14)
        self.assertEqual(app.data.time, paused_time)
        app.commands.put(("reset", None))
        app.advance(0)
        self.assertEqual(app.data.time, 0)
        np.testing.assert_allclose(app.data.qpos, initial, atol=1e-12)
        old_nodes = [node for _, node in app.renderer.nodes]
        app.load_scene("empty")
        self.assertEqual(app.environment.name, "Empty")
        self.assertEqual((app.model.nu, app.model.nq), (0, 0))
        self.assertTrue(all(node.removed for node in old_nodes))
        app.load_scene(self.xml)
        self.assertEqual(app.model.nu, 0)
        self.assertIsNone(app.arm)
        self.assertFalse(app.report()["robot_included"])

    def test_demo_classroom_preset_loads_native_assets_without_robot(self):
        model, data, environment = load_environment("demo_classroom", include_robot=False)
        self.assertEqual(environment.name, "demo_classroom")
        self.assertEqual((model.nu, model.nq), (0, 0))
        self.assertEqual(model.ntex, 4)
        self.assertEqual(sum(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE), 1)
        for index in range(4):
            self.assertGreater(model.body(f"object_{index:03d}").id, 0)
            self.assertGreater(data.xpos[model.body(f"object_{index:03d}").id, 0], 2)
        self.assertTrue(np.isfinite(data.geom_xpos).all())


if __name__ == "__main__":
    unittest.main()
