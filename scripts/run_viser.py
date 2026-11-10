"""Browser-based MeArm simulator: MuJoCo physics, Viser controls and rendering."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import time

ROOT = Path(__file__).resolve().parents[1]
if __package__:
    from ._runtime import configure_dependencies
else:
    from _runtime import configure_dependencies
configure_dependencies(ROOT)

import mujoco
import numpy as np
from PIL import Image
import trimesh

if __package__:
    from .arm_control import MeArmController
    from .scene_environment import PRESETS, add_scene_arguments, load_environment, scene_kwargs
    from .cube_control import CubeSceneController
    from .cube_scene import CUBES
    from .pick_place import PickPlaceTask
else:
    from arm_control import MeArmController
    from scene_environment import PRESETS, add_scene_arguments, load_environment, scene_kwargs
    from cube_control import CubeSceneController
    from cube_scene import CUBES
    from pick_place import PickPlaceTask


def configure_viser_lighting(server):
    """Keep the visible background empty without removing environment light.

    MuJoCo lights are not copied into Viser. Explicit fill lights prevent dark
    faces, and shadow-free lighting avoids cascaded-shadow acne at MeArm scale.
    The bundled studio HDRI illuminates imported materials but is never shown.
    """
    server.scene.set_up_direction("+z")
    server.scene.configure_environment_map(hdri="studio", background=False, environment_intensity=.8)
    server.scene.configure_default_lights(enabled=False, cast_shadow=False)
    ambient = server.scene.add_light_ambient("/lighting/ambient", intensity=.8)
    key = server.scene.add_light_directional(
        "/lighting/key", intensity=1.8, cast_shadow=False,
        wxyz=trimesh.transformations.quaternion_from_euler(*np.deg2rad([30, -40, -25])))
    fill = server.scene.add_light_directional(
        "/lighting/fill", intensity=.6, cast_shadow=False,
        wxyz=trimesh.transformations.quaternion_from_euler(*np.deg2rad([-40, 35, 130])))
    return ambient, key, fill


class PhysicsSceneRenderer:
    """Render compiled MuJoCo visual geometry at its actual world pose.

    Mesh vertices use MuJoCo's compiled mesh frame, not the STL source frame;
    geom_xmat/geom_xpos include the compiler's mesh recentering automatically.
    Background imports retain their original texture/materials through GLB.
    """

    def __init__(self, server, model, data, environment):
        self.server = server
        self.nodes = []
        self.static_nodes = []
        self.site_nodes = []
        self.model = model
        self.environment = environment
        self._visible_bounds = []
        for index in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or str(index)
            if name in environment.visual_meshes or model.geom_group[index] >= 3:
                continue
            rgba = np.asarray(model.geom_rgba[index]).copy()
            mat_id = model.geom_matid[index]
            if mat_id >= 0:
                rgba = np.asarray(model.mat_rgba[mat_id]).copy()
            if rgba[3] <= 0:
                continue
            mesh = self.geometry_mesh(model, index)
            if mesh is None:
                continue
            if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_PLANE:
                vertices = mesh.vertices @ data.geom_xmat[index].reshape(3, 3).T + data.geom_xpos[index]
                self._visible_bounds.append(np.array([vertices.min(axis=0), vertices.max(axis=0)]))
            textured = self.textured_mesh(model, index, mesh, rgba)
            if textured is not None:
                node = server.scene.add_mesh_trimesh(
                    "/mearm/" + name, textured, cast_shadow=False, receive_shadow=False)
            else:
                node = server.scene.add_mesh_simple(
                    "/mearm/" + name, vertices=np.asarray(mesh.vertices, dtype=np.float32),
                    faces=np.asarray(mesh.faces, dtype=np.uint32),
                    color=tuple(int(v) for v in np.clip(rgba[:3] * 255, 0, 255)),
                    opacity=float(rgba[3]), side="double", flat_shading=True,
                    cast_shadow=False, receive_shadow=False)
            self.nodes.append((index, node))
        for name, mesh in environment.visual_meshes.items():
            self._visible_bounds.append(mesh.bounds)
            self.static_nodes.append(server.scene.add_mesh_trimesh(
                "/environment/" + name, mesh, cast_shadow=False, receive_shadow=False))
        # These task markers change visibility and position as a job runs.
        for name in ("reach_target", "place_target"):
            try:
                sid = int(model.site(name).id)
            except (KeyError, ValueError):
                continue
            if model.site_type[sid] == mujoco.mjtGeom.mjGEOM_BOX:
                mesh = trimesh.creation.box(extents=2 * model.site_size[sid])
            else:
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=model.site_size[sid, 0])
            rgba = model.site_rgba[sid]
            node = server.scene.add_mesh_simple(
                "/targets/" + name, vertices=np.asarray(mesh.vertices, dtype=np.float32),
                faces=np.asarray(mesh.faces, dtype=np.uint32),
                color=tuple(int(v) for v in np.clip(rgba[:3] * 255, 0, 255)),
                opacity=float(rgba[3]) if rgba[3] > 0 else .35,
                visible=bool(rgba[3] > 0), side="double", cast_shadow=False, receive_shadow=False)
            self.site_nodes.append((sid, node))
        background = np.array(environment.definition.get("background_rgb", [.92, .94, .96]))
        server.scene.set_background_image(np.round(background[None, None, :] * 255).astype(np.uint8))
        self.sync(data)

    @staticmethod
    def textured_mesh(model, index, mesh, rgba):
        """Recover native MJCF UVs without mixing source and compiled mesh frames."""
        if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_MESH:
            return None
        material_id = int(model.geom_matid[index])
        if material_id < 0:
            return None
        slots = np.asarray(model.mat_texid[material_id]).ravel()
        # MuJoCo 3.10 uses texture-role slots; older models use one texture ID.
        slot = int(mujoco.mjtTextureRole.mjTEXROLE_RGB) if hasattr(mujoco, "mjtTextureRole") else 0
        texture_id = int(slots[slot if len(slots) > 1 else 0])
        if texture_id < 0 or model.tex_type[texture_id] != mujoco.mjtTexture.mjTEXTURE_2D:
            return None
        mid = int(model.geom_dataid[index])
        uv_start, uv_count = int(model.mesh_texcoordadr[mid]), int(model.mesh_texcoordnum[mid])
        if uv_start < 0 or uv_count == 0:
            return None
        face_start, face_count = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
        uv_faces = model.mesh_facetexcoord[face_start:face_start + face_count]
        if np.any(uv_faces < 0):
            return None
        # OBJ texture seams have independent vertex/UV indices. Splitting by
        # the pair retains both geometry and face-average texture atlases.
        pairs, inverse = np.unique(np.column_stack((mesh.faces.ravel(), uv_faces.ravel())),
                                   axis=0, return_inverse=True)
        width, height = int(model.tex_width[texture_id]), int(model.tex_height[texture_id])
        channels = int(model.tex_nchannel[texture_id]) if hasattr(model, "tex_nchannel") else 3
        start = int(model.tex_adr[texture_id])
        pixels = model.tex_data if hasattr(model, "tex_data") else model.tex_rgb
        pixels = pixels[start:start + width * height * channels].reshape(height, width, channels)
        image = Image.fromarray(pixels.copy())
        uv = model.mesh_texcoord[uv_start:uv_start + uv_count][pairs[:, 1]].copy()
        # MuJoCo stores top-left texture coordinates; Trimesh expects authored
        # bottom-left UVs and converts them to glTF conventions during export.
        uv[:, 1] = 1 - uv[:, 1]
        visual = trimesh.visual.texture.TextureVisuals(
            uv=uv,
            material=trimesh.visual.material.PBRMaterial(
                baseColorTexture=image, baseColorFactor=np.round(np.clip(rgba, 0, 1) * 255).astype(np.uint8),
                doubleSided=True, metallicFactor=0., roughnessFactor=1.))
        return trimesh.Trimesh(vertices=mesh.vertices[pairs[:, 0]],
                               faces=inverse.reshape(-1, 3), visual=visual, process=False)

    def camera_view(self):
        """Frame native scenes at their saved world location, excluding planes."""
        if not self.environment.definition.get("mjcf") or not self._visible_bounds:
            return dict(position=np.array([-.42, -.40, .30]), look_at=np.array([.07, 0, .10]),
                        near=.001, far=100.)
        bounds = np.asarray(self._visible_bounds)
        lower, upper = bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)
        center = (lower + upper) / 2
        radius = max(float(np.linalg.norm(upper - lower)) / 2, .1)
        direction = np.array([-.8, -1., .65])
        direction /= np.linalg.norm(direction)
        return dict(position=center + direction * radius * 3.5, look_at=center,
                    near=max(.001, radius / 1000), far=max(100., radius * 20))

    @staticmethod
    def geometry_mesh(model, index):
        kind = model.geom_type[index]
        size = model.geom_size[index]
        types = mujoco.mjtGeom
        if kind == types.mjGEOM_MESH:
            mid = int(model.geom_dataid[index])
            va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
            fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
            return trimesh.Trimesh(vertices=model.mesh_vert[va:va+vn].copy(),
                                   faces=model.mesh_face[fa:fa+fn].copy(), process=False)
        if kind == types.mjGEOM_BOX:
            return trimesh.creation.box(extents=2 * size)
        if kind == types.mjGEOM_PLANE:
            # Finite display surface, unlike MuJoCo's mathematically infinite
            # collision plane. Its upper face is exactly at the plane's Z=0.
            mesh = trimesh.creation.box(extents=(max(2*size[0], 2), max(2*size[1], 2), .002))
            mesh.apply_translation([0, 0, -.001])
            return mesh
        if kind == types.mjGEOM_SPHERE:
            return trimesh.creation.icosphere(subdivisions=2, radius=size[0])
        if kind == types.mjGEOM_CYLINDER:
            return trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=32)
        if kind == types.mjGEOM_CAPSULE:
            return trimesh.creation.capsule(radius=size[0], height=2 * size[1])
        return None

    def sync(self, data):
        with self.server.atomic():
            for index, node in self.nodes:
                quaternion = np.empty(4)
                mujoco.mju_mat2Quat(quaternion, data.geom_xmat[index])
                node.wxyz = quaternion
                node.position = data.geom_xpos[index].copy()
            for index, node in self.site_nodes:
                quaternion = np.empty(4)
                mujoco.mju_mat2Quat(quaternion, data.site_xmat[index])
                node.wxyz = quaternion
                node.position = data.site_xpos[index].copy()
                alpha = float(self.model.site_rgba[index, 3])
                node.visible = alpha > 0
                if alpha > 0:
                    node.opacity = alpha

    def remove(self):
        for _, node in self.nodes:
            node.remove()
        for node in self.static_nodes:
            node.remove()
        for _, node in self.site_nodes:
            node.remove()


class ViserSimulation:
    """GUI callbacks queue commands; only the simulation thread edits physics."""

    def __init__(self, server, scene="empty", *, task="arm", scene_options=None, demo=False,
                 include_robot=True):
        if not include_robot and (task != "arm" or demo):
            raise ValueError("--no-robot cannot be combined with --task cubes or --demo.")
        self.server = server
        self.task = task
        self.include_robot = include_robot
        self.scene_options = scene_options or {}
        self.commands = queue.Queue()
        self.renderer = None
        self.scene_reference = str(scene)
        self.demo = demo
        self.paused = False
        self._setting_widgets = False
        self.lights = configure_viser_lighting(server)
        self.load_scene(scene)
        self.make_gui()

    def load_scene(self, reference):
        if getattr(self, "pick_task", None) is not None and self.pick_task.active:
            raise ValueError("Cancel pick and place before changing the scene.")
        base_xml = None
        if self.task == "cubes":
            if __package__:
                from .cube_scene import build_scene_xml
            else:
                from cube_scene import build_scene_xml
            base_xml = build_scene_xml(grasp_ready=True)
        model, data, environment = load_environment(reference, base_xml=base_xml,
                                                    include_robot=self.include_robot, **self.scene_options)
        motion = CubeSceneController(model, data) if self.task == "cubes" else None
        arm = (motion.arm if motion is not None else MeArmController(model, data)) if self.include_robot else None
        if arm is not None and getattr(self, "arm", None) is not None:
            arm.set_joint_targets(*self.arm.target_degrees)
        # Import/compile must succeed before removing the current scene.
        if self.renderer is not None:
            self.renderer.remove()
        self.model, self.data, self.environment, self.arm = model, data, environment, arm
        self.motion = motion
        self.pick_task = PickPlaceTask(motion) if motion is not None else None
        self.scene_reference = str(reference)
        self.renderer = PhysicsSceneRenderer(self.server, model, data, environment)
        for client in getattr(self.server, "get_clients", lambda: {})().values():
            self.configure_client_camera(client)

    def configure_client_camera(self, client):
        for key, value in self.renderer.camera_view().items():
            setattr(client.camera, key, value)
        client.camera.up_direction = (0, 0, 1)

    def make_gui(self):
        gui = self.server.gui
        gui.configure_theme(dark_mode=False)
        gui.set_panel_label("MeArm simulation")
        gui.add_markdown("**MuJoCo physics · Viser viewer**\n\n" + (
            "Joint angles: degrees. Tool targets: millimetres. X forward, Y left, Z up."
            if self.include_robot else "Environment only. Coordinates: metres, Z up."))
        with gui.add_folder("Environment"):
            self.preset_dropdown = gui.add_dropdown(
                "Preset", (*PRESETS, "Custom file"),
                initial_value=self.scene_reference if self.scene_reference in PRESETS else "Custom file")
            self.preset_dropdown.on_update(lambda event: self.commands.put(("scene", event.target.value))
                                          if event.target.value in PRESETS else None)
            self.scene_path = gui.add_text("Scene / MJCF / mesh / image path", initial_value=self.scene_reference)
            self.load_button = gui.add_button("Load scene")
            self.load_button.on_click(lambda _: self.commands.put(("scene", self.scene_path.value)))
            gui.add_markdown("Paths are read on the Python server. Native MJCF retains its geometry, materials and collisions.")
        self.sliders = []
        if self.arm is not None:
            with gui.add_folder("Joint controls"):
                labels = ("Yaw", "Shoulder elevation", "Forearm elevation", "Jaw opening")
                for index, (label, bounds, value) in enumerate(zip(labels, self.arm.limits_degrees, self.arm.target_degrees)):
                    slider = gui.add_slider(label, min=float(bounds[0]), max=float(bounds[1]),
                                            step=.1, initial_value=float(value))
                    def on_joint(event, joint_index=index):
                        if not self._setting_widgets:
                            self.commands.put(("joint", (joint_index, float(event.target.value))))
                    slider.on_update(on_joint)
                    self.sliders.append(slider)
                self.home_button = gui.add_button("Home")
                self.home_button.on_click(lambda _: self.commands.put(("home", None)))
            with gui.add_folder("Tool target"):
                self.xyz = [gui.add_number(axis + " (mm)", initial_value=value, step=1.)
                            for axis, value in zip("XYZ", (148., 0., 135.))]
                self.reach_button = gui.add_button("Reach target")
                self.reach_button.on_click(lambda _: self.commands.put(("xyz", [field.value for field in self.xyz])))
        if self.task == "cubes":
            with gui.add_folder("Pick and place"):
                self.cube_color = gui.add_dropdown("Cube color", [cube["name"] for cube in CUBES], initial_value="red")
                self.place_xyz = [gui.add_number("Place " + axis + " (mm)", initial_value=value, step=1.)
                                  for axis, value in zip("XYZ", (180., -45., 90.))]
                self.move_seconds = gui.add_number("Move duration (s)", initial_value=2., min=.5, step=.1)
                self.clearance_mm = gui.add_number("Lift clearance (mm)", initial_value=35., min=20., step=1.)
                self.pick_button = gui.add_button("Pick and place selected cube")
                self.pick_button.on_click(lambda _: self.commands.put(("pick", (
                    self.cube_color.value, [field.value for field in self.place_xyz],
                    self.move_seconds.value, self.clearance_mm.value))))
                self.cancel_pick_button = gui.add_button("Cancel pick and place")
                self.cancel_pick_button.on_click(lambda _: self.commands.put(("cancel_pick", None)))
                self.pick_status = gui.add_text("Pick/place progress", initial_value="idle", disabled=True)
                self.cube_positions = gui.add_text("Cube centers (mm)", initial_value="", multiline=True, disabled=True)
                gui.add_markdown("20 mm cubes. Place XYZ is the **cube center**; use Z=90 on this tabletop. "
                                 "The green square marks the destination. Pause freezes the sequence; Cancel returns joint control.")
        with gui.add_folder("Playback"):
            if self.arm is not None:
                self.demo_checkbox = gui.add_checkbox("Joint demo", initial_value=self.demo)
                self.demo_checkbox.on_update(lambda event: self.commands.put(("demo", bool(event.target.value))))
            self.pause_checkbox = gui.add_checkbox("Pause physics", initial_value=False)
            self.pause_checkbox.on_update(lambda event: self.commands.put(("pause", bool(event.target.value))))
            self.reset_button = gui.add_button("Reset physics")
            self.reset_button.on_click(lambda _: self.commands.put(("reset", None)))
        self.status = gui.add_text("Status", initial_value="Ready: " + self.environment.name, disabled=True)
        self.tcp = gui.add_text("Current tool XYZ (mm)", initial_value="", disabled=True) if self.arm is not None else None
        self.refresh_pick_panel()

    def start_pick_place(self, color, destination_mm, move_seconds=2., clearance_mm=35.):
        if self.pick_task is None:
            raise ValueError("Pick and place needs the colored-cube task; launch with --task cubes.")
        self.pick_task.start(color, destination_mm, move_seconds, clearance_mm)
        self.demo = False
        if hasattr(self, "status"):
            self.status.value = "Picking " + self.pick_task.color
            self.update_widgets()
            self.refresh_pick_panel()

    def take_manual_control(self):
        if self.motion is not None:
            self.motion.set_manual()
            if self.pick_task.color is not None:
                self.pick_task.reset()

    def refresh_pick_panel(self):
        if self.pick_task is None or not hasattr(self, "pick_status"):
            return
        result = self.pick_task.report()
        progress = f"{result['status']}: {result['phase']}"
        if result['color'] is not None:
            progress += " | " + result['color']
        if self.paused:
            progress += " | paused"
        if result['status'] == 'complete':
            progress += f" | error {result['placement_error_mm']:.2f} mm | released={result['released']}"
        if result['error']:
            progress += " | " + result['error']
        self.pick_status.value = progress
        self.cube_positions.value = "\n".join(
            cube['name'] + ": " + ", ".join(f"{v:.1f}" for v in
                1000 * self.data.xpos[self.model.body('cube_' + cube['name']).id]) for cube in CUBES)
        busy = self.pick_task.active
        for control in [*self.sliders, self.home_button, self.reach_button, self.demo_checkbox,
                        self.load_button, self.preset_dropdown, self.pick_button, self.cube_color,
                        *self.place_xyz, self.move_seconds, self.clearance_mm]:
            control.disabled = busy
        self.cancel_pick_button.disabled = not busy

    def update_widgets(self):
        self._setting_widgets = True
        try:
            with self.server.atomic():
                if self.arm is not None:
                    for slider, value in zip(self.sliders, self.arm.target_degrees):
                        slider.value = float(value)
                    self.demo_checkbox.value = self.demo
                self.preset_dropdown.value = self.scene_reference if self.scene_reference in PRESETS else "Custom file"
        finally:
            self._setting_widgets = False

    def process_commands(self):
        while not self.commands.empty():
            command, value = self.commands.get_nowait()
            try:
                if self.arm is None and command in ("joint", "home", "xyz", "demo"):
                    raise ValueError("Robot controls are unavailable with --no-robot.")
                if self.pick_task is not None and self.pick_task.active and command not in ("pause", "cancel_pick", "reset"):
                    raise ValueError("Pick and place is running. Cancel it before changing joints, targets or scenes.")
                if command == "scene":
                    self.load_scene(value)
                    self.scene_path.value = self.scene_reference
                elif command == "joint":
                    index, angle = value
                    fields = ("yaw_deg", "shoulder_deg", "forearm_deg", "grip_deg")
                    self.arm.set_joint_targets(**{fields[index]: angle})
                    self.take_manual_control()
                    self.demo = False
                elif command == "home":
                    self.arm.set_joint_targets(0, 90, 0, 0)
                    self.take_manual_control()
                    self.demo = False
                elif command == "xyz":
                    self.arm.move_to(*value)
                    self.take_manual_control()
                    self.demo = False
                elif command == "pick":
                    self.start_pick_place(*value)
                elif command == "cancel_pick":
                    if self.pick_task is not None:
                        self.pick_task.cancel()
                elif command == "demo":
                    self.take_manual_control()
                    self.demo = value
                elif command == "pause":
                    self.paused = value
                elif command == "reset":
                    if self.pick_task is not None:
                        self.pick_task.cancel()
                    mujoco.mj_resetData(self.model, self.data)
                    if self.motion is not None:
                        self.motion = CubeSceneController(self.model, self.data)
                        self.arm = self.motion.arm
                        self.pick_task = PickPlaceTask(self.motion)
                        self.model.site_rgba[self.model.site('place_target').id, 3] = 0
                    mujoco.mj_forward(self.model, self.data)
                self.status.value = "Ready: " + self.environment.name
                self.update_widgets()
                self.refresh_pick_panel()
            except (ValueError, OSError) as exc:
                self.status.value = "Rejected: " + str(exc)
                self.update_widgets()

    def advance(self, seconds):
        self.process_commands()
        was_picking = self.pick_task is not None and self.pick_task.active
        if not self.paused:
            for _ in range(max(0, round(seconds / self.model.opt.timestep))):
                if self.demo:
                    t = self.data.time
                    self.data.ctrl[self.arm.actuator_ids] = [
                        .40 * math.sin(.55*t), .32 * math.sin(.7*t),
                        .35 * math.sin(.7*t+1), .25+.22*math.sin(1.1*t)]
                if self.pick_task is not None:
                    self.pick_task.update()
                mujoco.mj_step(self.model, self.data)
        # mj_step leaves position-dependent data at the start of the last step.
        mujoco.mj_forward(self.model, self.data)
        self.renderer.sync(self.data)
        if self.tcp is not None:
            self.tcp.value = ", ".join(f"{v:.1f}" for v in self.arm.tcp_position_mm)
        if was_picking:
            self.update_widgets()
        self.refresh_pick_panel()

    def report(self):
        return {"scene": self.scene_reference, "scene_name": self.environment.name,
                "physics": "MuJoCo " + mujoco.__version__, "task": self.task if self.include_robot else "scene",
                "robot_included": self.include_robot,
                "simulation_seconds": float(self.data.time),
                "target_degrees": None if self.arm is None else self.arm.target_degrees.tolist(),
                "tcp_mm": None if self.arm is None else self.arm.tcp_position_mm.tolist(),
                "pick_place": None if self.pick_task is None else self.pick_task.report(),
                "visual_nodes": len(self.renderer.nodes) + len(self.renderer.static_nodes) + len(self.renderer.site_nodes),
                "warnings": [int(w.number) for w in self.data.warning]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_scene_arguments(parser)
    parser.add_argument("--host", default="127.0.0.1", help="Bind address; default local computer only")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--task", choices=("arm", "cubes"), default="arm", help="Cubes/platform are opt-in")
    parser.add_argument("--no-robot", action="store_true", help="Load only the environment, without MeArm or robot controls")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--open-browser", action="store_true", help="Open the actual local server URL in your default browser")
    parser.add_argument("--cube", choices=[cube['name'] for cube in CUBES], help="Start an automatic pick; requires --task cubes --place-mm X Y Z")
    parser.add_argument("--place-mm", type=float, nargs=3, metavar=("X", "Y", "Z"), help="Destination cube CENTER in world mm; tabletop Z=90")
    parser.add_argument("--move-seconds", type=float, default=2., help="Pick/place motion segment duration")
    parser.add_argument("--clearance-mm", type=float, default=35., help="Pick/place lift clearance")
    parser.add_argument("--seconds", type=float, help="Stop after this many wall-clock seconds (for smoke checks)")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args(argv)
    if args.no_robot and (args.task != "arm" or args.demo):
        parser.error("--no-robot cannot be combined with --task cubes or --demo.")
    if (args.cube is None) != (args.place_mm is None):
        parser.error("Use --cube COLOR together with --place-mm X Y Z.")
    if args.cube is not None and (args.task != "cubes" or args.demo):
        parser.error("Automatic pick/place requires --task cubes and cannot be combined with --demo.")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535.")
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be positive and finite.")
    try:
        import viser
    except ImportError:
        parser.exit(1, "Viser is not installed. Run: python -m pip install -r requirements-viser.txt\n")
    server = viser.ViserServer(host=args.host, port=args.port, label="MeArm")
    app = None
    try:
        app = ViserSimulation(server, args.scene, task=args.task, scene_options=scene_kwargs(args),
                              demo=args.demo, include_robot=not args.no_robot)
        if args.cube is not None:
            app.cube_color.value = args.cube
            for widget, value in zip(app.place_xyz, args.place_mm):
                widget.value = float(value)
            app.move_seconds.value = args.move_seconds
            app.clearance_mm.value = args.clearance_mm
            app.start_pick_place(args.cube, args.place_mm, args.move_seconds, args.clearance_mm)

        @server.on_client_connect
        def on_connect(client):
            app.configure_client_camera(client)

        print(f"MeArm Viser: http://{args.host}:{server.get_port()}", flush=True)
        print(f"Scene: {app.environment.name}; physics: MuJoCo; Ctrl+C stops the server.", flush=True)
        if args.open_browser:
            import webbrowser
            browser_host = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
            try:
                if not webbrowser.open_new_tab(f"http://{browser_host}:{server.get_port()}"):
                    print("Open the printed URL manually if no browser tab appeared.", flush=True)
            except (OSError, webbrowser.Error) as exc:
                print(f"Could not open a browser ({exc}); open the printed URL manually.", flush=True)
        started = time.monotonic()
        previous = started
        accumulator = 0.
        while args.seconds is None or time.monotonic() - started < args.seconds:
            tick = time.monotonic()
            accumulator += min(.1, tick - previous)
            previous = tick
            steps = int(accumulator / app.model.opt.timestep)
            elapsed = steps * app.model.opt.timestep
            app.advance(elapsed)
            accumulator -= elapsed
            time.sleep(max(0, 1/30 - (time.monotonic() - tick)))
    except (ValueError, OSError) as exc:
        parser.exit(1, "Cannot launch scene: " + str(exc) + "\n")
    except KeyboardInterrupt:
        print("\nStopping MeArm Viser.")
    finally:
        if args.report and app is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(app.report(), indent=2) + "\n", encoding="utf-8")
        server.stop()


if __name__ == "__main__":
    main()
