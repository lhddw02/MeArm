"""Scene geometry, transforms, physics isolation and Viser mesh-frame checks."""
from contextlib import nullcontext
import json
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from PIL import Image
import trimesh

from scripts.arm_control import MeArmController
from scripts.run_viser import PhysicsSceneRenderer, configure_viser_lighting
from scripts.scene_environment import ROOT, load_environment, transform_matrix


class FakeNode:
    def __init__(self, kwargs):
        self.kwargs = kwargs
        self.wxyz = np.array([1., 0., 0., 0.])
        self.position = np.zeros(3)
        self.removed = False

    def remove(self):
        self.removed = True


class FakeSceneApi:
    def __init__(self):
        self.nodes = {}

    def add_mesh_simple(self, name, **kwargs):
        node = FakeNode(kwargs)
        self.nodes[name] = node
        return node

    def add_mesh_trimesh(self, name, mesh, **kwargs):
        node = FakeNode({"mesh": mesh, **kwargs})
        self.nodes[name] = node
        return node

    def set_background_image(self, image):
        self.image = image

    def set_up_direction(self, direction):
        self.up_direction = direction

    def configure_environment_map(self, **kwargs):
        self.environment_lighting = kwargs

    def configure_default_lights(self, **kwargs):
        self.default_lights = kwargs

    def add_light_ambient(self, name, **kwargs):
        node = FakeNode(kwargs)
        self.nodes[name] = node
        return node

    def add_light_directional(self, name, **kwargs):
        return self.add_light_ambient(name, **kwargs)


class FakeServer:
    def __init__(self):
        self.scene = FakeSceneApi()

    def atomic(self):
        return nullcontext()


class SceneEnvironmentTests(unittest.TestCase):
    def setUp(self):
        (ROOT / "tmp").mkdir(exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=ROOT / "tmp")
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)

    def write_json(self, objects, **kwargs):
        path = self.folder / "my scene.json"
        path.write_text(json.dumps({"version": 1, "objects": objects, **kwargs}))
        return path

    def test_empty_is_default_and_keeps_only_arm_and_support_floor(self):
        model, data, environment = load_environment()
        stock = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/mearm.xml"))
        self.assertEqual(environment.name, "Empty")
        self.assertEqual(environment.visual_meshes, {})
        self.assertEqual(model.ngeom, stock.ngeom)
        self.assertEqual(model.nq, stock.nq)
        self.assertEqual(model.nu, 4)
        np.testing.assert_array_equal(model.actuator_ctrlrange, stock.actuator_ctrlrange)
        self.assertEqual(model.mat_texid[model.material("floor_mat").id].max(), -1)
        self.assertEqual(len(data.qpos), 9)

    def test_visual_backgrounds_do_not_change_arm_physics_or_fk(self):
        empty, empty_data, _ = load_environment()
        scene, scene_data, _ = load_environment("workshop")
        original = MeArmController(empty, empty_data)
        decorated = MeArmController(scene, scene_data)
        for arm in (original, decorated):
            arm.set_joint_targets(20, 75, -15, 10)
        for _ in range(300):
            mujoco.mj_step(empty, empty_data)
            mujoco.mj_step(scene, scene_data)
        np.testing.assert_allclose(scene_data.qpos, empty_data.qpos, atol=1e-12)
        np.testing.assert_allclose(scene_data.qvel, empty_data.qvel, atol=1e-12)
        np.testing.assert_allclose(original.tcp_position_mm, decorated.tcp_position_mm, atol=1e-10)
        extras = [i for i in range(scene.ngeom) if scene.geom(i).name.startswith("environment_")]
        self.assertEqual(len(extras), 6)
        self.assertTrue(all(scene.geom_contype[i] == 0 and scene.geom_conaffinity[i] == 0 for i in extras))

    def test_mesh_instances_unit_scale_rotation_and_placement(self):
        mesh = trimesh.creation.box(extents=[20, 40, 60])  # Imported millimetres.
        source = trimesh.Scene()
        source.add_geometry(mesh, node_name="one", transform=trimesh.transformations.translation_matrix([100, 0, 0]))
        source.add_geometry(mesh, node_name="two", transform=trimesh.transformations.translation_matrix([0, 200, 0]))
        path = self.folder / "room with spaces.glb"
        path.write_bytes(source.export(file_type="glb"))
        pose = transform_matrix([.3, -.2, .1], [0, 0, 90], .001)
        model, _, environment = load_environment(path, scale=.001, position=[.3, -.2, .1], rpy=[0, 0, 90])
        self.assertEqual(len(environment.visual_meshes), 2)
        expected = []
        for node in source.graph.nodes_geometry:
            transform, geometry_name = source.graph[node]
            vertices = trimesh.transform_points(source.geometry[geometry_name].vertices, pose @ transform)
            expected.extend(vertices)
        actual = np.concatenate([mesh.vertices for mesh in environment.visual_meshes.values()])
        np.testing.assert_allclose([actual.min(axis=0), actual.max(axis=0)],
                                   [np.min(expected, axis=0), np.max(expected, axis=0)], atol=1e-7)
        self.assertEqual(model.nu, 4)

    def test_image_and_flat_textured_mesh_import_work_in_mujoco_and_viser(self):
        path = self.folder / "backdrop.png"
        image = np.zeros((16, 32, 3), dtype=np.uint8)
        image[:, :16] = [255, 0, 0]
        image[:, 16:] = [0, 0, 255]
        Image.fromarray(image).save(path)
        model, data, environment = load_environment(path)
        self.assertEqual(len(environment.visual_meshes), 1)
        self.assertEqual(len(environment.assets), 1)
        self.assertEqual(model.ntex, 2)  # Flat sky and imported image.
        server = FakeServer()
        renderer = PhysicsSceneRenderer(server, model, data, environment)
        self.assertEqual(len(renderer.static_nodes), 1)
        self.assertNotIn("/mearm/environment_imported_0", server.scene.nodes)
        for mesh in environment.visual_meshes.values():
            self.assertEqual(mesh.visual.kind, "texture")
            self.assertEqual(mesh.visual.uv.shape, (4, 2))

    def test_obj_stl_ply_and_gltf_keep_geometry_without_extra_dependencies(self):
        source = trimesh.creation.box(extents=[.05, .08, .12])
        for suffix in ("obj", "stl", "ply", "gltf"):
            with self.subTest(suffix=suffix):
                if suffix == "gltf":
                    exported = trimesh.Scene(source).export(file_type="gltf")
                    for name, contents in exported.items():
                        (self.folder / name).write_bytes(contents)
                    path = self.folder / next(name for name in exported if name.endswith(".gltf"))
                else:
                    path = self.folder / ("model." + suffix)
                    source.export(path)
                model, _, environment = load_environment(path)
                self.assertEqual(len(environment.visual_meshes), 1)
                mesh = next(iter(environment.visual_meshes.values()))
                np.testing.assert_allclose(mesh.bounds, source.bounds, atol=1e-7)
                self.assertEqual(model.nu, 4)

    def test_explicit_collision_proxies_and_transform_validation(self):
        path = self.write_json([{"name": "obstacle", "type": "box", "size_m": [.1, .1, .1],
                                 "pos_m": [.3, 0, .05], "collidable": True}])
        model, _, _ = load_environment(path)
        geom = model.geom("environment_obstacle").id
        self.assertEqual(model.geom_contype[geom], 1)
        np.testing.assert_allclose(model.geom_pos[geom], [.3, 0, .05])
        for scale in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                load_environment(path, scale=scale)

    def test_invalid_or_missing_scene_fails_without_silent_fallback(self):
        with self.assertRaisesRegex(ValueError, "Scene not found"):
            load_environment(self.folder / "does-not-exist.glb")
        cases = [
            [{"name": "x", "type": "unknown"}],
            [{"name": "x", "type": "box", "size_m": [1, -1, 1]}],
            [{"name": "x", "type": "mesh", "file": "missing.glb"}],
            [{"name": "x", "type": "box", "size_m": [1, 1, 1], "collidable": "false"}],
            [{"name": "x", "type": "box", "size_m": [1, 1, 1], "unexpected": 1}],
        ]
        for objects in cases:
            with self.subTest(objects=objects), self.assertRaises(ValueError):
                load_environment(self.write_json(objects))

    def test_viser_dynamic_mesh_frame_matches_mujoco_world_vertices(self):
        model, data, environment = load_environment("classroom")
        arm = MeArmController(model, data)
        arm.set_joint_targets(-20, 65, 20, 15)
        for _ in range(300):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        server = FakeServer()
        renderer = PhysicsSceneRenderer(server, model, data, environment)
        for geom_id, node in renderer.nodes:
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, node.wxyz)
            np.testing.assert_allclose(rotation, data.geom_xmat[geom_id], atol=1e-12)
            np.testing.assert_allclose(node.position, data.geom_xpos[geom_id], atol=1e-12)
            if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = model.geom_dataid[geom_id]
                start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                np.testing.assert_array_equal(node.kwargs["vertices"], model.mesh_vert[start:start+count])
        renderer.remove()
        self.assertTrue(all(node.removed for node in server.scene.nodes.values()))

    def test_viser_lighting_has_fill_without_background_or_shadow_acne(self):
        server = FakeServer()
        ambient, key, fill = configure_viser_lighting(server)
        self.assertGreater(ambient.kwargs["intensity"], 0)
        self.assertFalse(server.scene.environment_lighting["background"])
        self.assertIsNotNone(server.scene.environment_lighting["hdri"])
        self.assertFalse(server.scene.default_lights["enabled"])
        self.assertFalse(server.scene.default_lights["cast_shadow"])
        for light in (key, fill):
            self.assertGreater(light.kwargs["intensity"], 0)
            self.assertFalse(light.kwargs["cast_shadow"])
        model, data, environment = load_environment(ROOT / "scenes/imported_room.json")
        renderer = PhysicsSceneRenderer(server, model, data, environment)
        for node in [node for _, node in renderer.nodes] + renderer.static_nodes:
            self.assertFalse(node.kwargs["cast_shadow"])
            self.assertFalse(node.kwargs["receive_shadow"])


if __name__ == "__main__":
    unittest.main()
