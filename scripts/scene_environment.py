"""Shared, static scene imports for MuJoCo and the physics-backed Viser viewer.

All coordinates are metres, Z up. Backgrounds never change the arm's joints,
actuators, or base frame. Imported meshes are visual-only; simple primitives
can explicitly enable collision. No user scene is executed as Python code.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import io
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

if __package__:
    from ._runtime import configure_dependencies
else:
    from _runtime import configure_dependencies
ROOT = Path(__file__).resolve().parents[1]
configure_dependencies(ROOT)
import mujoco
import numpy as np
from PIL import Image
import trimesh

MESH_EXTENSIONS = {".glb", ".gltf", ".obj", ".stl", ".ply"}
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}
PRESETS = ("empty", "workshop", "classroom")


def vector(value, length, label, *, positive=False):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain {length} finite numbers.") from exc
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {length} finite numbers.")
    if positive and np.any(result <= 0):
        raise ValueError(f"{label} must be positive.")
    return result


def scale_vector(value):
    if np.isscalar(value):
        value = [value] * 3
    return vector(value, 3, "Scene scale", positive=True)


def transform_matrix(position=(0, 0, 0), rpy=(0, 0, 0), scale=1):
    angles = np.deg2rad(vector(rpy, 3, "Scene rotation"))
    matrix = trimesh.transformations.euler_matrix(*angles, axes="sxyz")
    matrix[:3, :3] = matrix[:3, :3] @ np.diag(scale_vector(scale))
    matrix[:3, 3] = vector(position, 3, "Scene position")
    return matrix


def _numbers(values):
    return " ".join(f"{float(value):.12g}" for value in np.asarray(values).ravel())


def _rgba(value, label):
    result = vector(value, 4, label)
    if np.any(result < 0) or np.any(result > 1):
        raise ValueError(f"{label} must be within [0, 1].")
    return result


@dataclass
class Environment:
    name: str
    definition: dict
    directory: Path
    # Viser renders these original textured meshes; MuJoCo uses matching MJCF.
    visual_meshes: dict = field(default_factory=dict)
    assets: dict = field(default_factory=dict)


def read_environment(reference="empty"):
    reference = "empty" if reference is None else str(reference)
    candidate = Path(reference).expanduser()
    if reference in PRESETS:
        candidate = ROOT / "scenes" / (reference + ".json")
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise ValueError(f"Scene not found: {reference}. Choose {', '.join(PRESETS)}, "
                         "a scene JSON, a GLB/GLTF/OBJ/STL/PLY, or a PNG/JPG.")
    suffix = candidate.suffix.lower()
    if suffix == ".json":
        try:
            definition = json.loads(candidate.read_text(encoding="utf-8-sig"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"Cannot read scene JSON: {exc}") from exc
    elif suffix in MESH_EXTENSIONS | IMAGE_EXTENSIONS:
        definition = {"version": 1, "name": candidate.stem, "objects": [
            {"name": "imported", "type": "mesh" if suffix in MESH_EXTENSIONS else "image",
             "file": str(candidate)}]}
    else:
        raise ValueError(f"Unsupported scene file: {suffix}")
    if not isinstance(definition, dict) or definition.get("version") != 1:
        raise ValueError("Scene JSON needs version=1 and an objects array.")
    allowed = {"version", "name", "background_rgb", "floor_rgba", "objects"}
    if set(definition) - allowed:
        raise ValueError("Unknown scene fields: " + ", ".join(sorted(set(definition) - allowed)))
    if not isinstance(definition.get("objects"), list):
        raise ValueError("Scene objects must be an array.")
    color = vector(definition.get("background_rgb", [.92, .94, .96]), 3, "Background RGB")
    if np.any(color < 0) or np.any(color > 1):
        raise ValueError("Background RGB must be within [0, 1].")
    _rgba(definition.get("floor_rgba", [.88, .90, .92, 1]), "Floor RGBA")
    return Environment(str(definition.get("name", candidate.stem)), definition, candidate.parent)


def _load_meshes(path):
    if path.suffix.lower() not in MESH_EXTENSIONS:
        raise ValueError(f"Unsupported mesh format: {path.suffix}")
    try:
        scene = trimesh.load_scene(str(path), process=False)
    except Exception as exc:
        raise ValueError(f"Cannot import mesh {path}: {exc}") from exc
    parts = []
    for node in scene.graph.nodes_geometry:
        pose, geometry_name = scene.graph[node]
        mesh = scene.geometry[geometry_name].copy()
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            continue
        mesh.apply_transform(pose)
        # One MuJoCo material per geom: split GLTF multi-material primitives.
        material = getattr(mesh.visual, "material", None)
        if hasattr(material, "materials"):
            face_materials = np.asarray(mesh.visual.face_materials)
            for index, item in enumerate(material.materials):
                faces = np.flatnonzero(face_materials == index)
                if len(faces):
                    part = mesh.submesh([faces], append=True)
                    part.visual.material = item
                    parts.append(part)
        else:
            parts.append(mesh)
    if not parts:
        raise ValueError(f"No triangle meshes were found in {path}.")
    return parts


def _mesh_color(mesh):
    visual = mesh.visual
    material = getattr(visual, "material", None)
    if material is not None:
        color = getattr(material, "baseColorFactor", None)
        if color is None:
            color = getattr(material, "diffuse", None)
        if color is not None:
            color = np.asarray(color, dtype=float)
            if np.max(color) > 1:
                color /= 255
            return np.r_[color[:3], color[3] if len(color) > 3 else 1]
    if visual.kind in ("face", "vertex"):
        return np.mean(visual.face_colors, axis=0) / 255
    return np.array([.65, .68, .72, 1])


def _mesh_texture(mesh):
    material = getattr(mesh.visual, "material", None)
    image = getattr(material, "image", None)
    if image is None:
        image = getattr(material, "baseColorTexture", None)
    uv = getattr(mesh.visual, "uv", None)
    if image is None or uv is None:
        return None, None
    return image.convert("RGB"), np.asarray(uv, dtype=float)


def _add_mesh(root, environment, name, mesh, color=None):
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    if not np.isfinite(vertices).all() or len(vertices) < 3 or len(faces) == 0:
        raise ValueError(f"Scene mesh {name} has invalid or empty geometry.")
    asset = root.find("asset")
    attrs = {"name": name + "_mesh", "vertex": _numbers(vertices),
             "face": " ".join(map(str, faces.ravel())), "inertia": "shell"}
    image, uv = _mesh_texture(mesh) if color is None else (None, None)
    if uv is not None:
        if uv.shape != (len(vertices), 2) or not np.isfinite(uv).all():
            raise ValueError(f"Scene mesh {name} has invalid texture coordinates.")
        attrs["texcoord"] = _numbers(uv)
    # Surface inertia supports thin room walls, even though these geoms have no
    # collision and contribute no mass to the arm's explicit link inertias.
    ET.SubElement(asset, "mesh", **attrs)
    rgba = _mesh_color(mesh) if color is None else color
    geom = dict(name=name, type="mesh", mesh=name + "_mesh", rgba=_numbers(rgba),
                contype="0", conaffinity="0", group="1")
    if image is not None:
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        filename = name + ".png"
        environment.assets[filename] = buffer.getvalue()
        ET.SubElement(asset, "texture", name=name + "_texture", type="2d", file=filename)
        ET.SubElement(asset, "material", name=name + "_material", texture=name + "_texture",
                      texuniform="false", texrepeat="1 1", rgba=_numbers(rgba))
        geom["material"] = name + "_material"
    ET.SubElement(root.find("worldbody"), "geom", **geom)
    if color is not None:
        # Match explicit material overrides in both renderers.
        mesh = mesh.copy()
        mesh.visual = trimesh.visual.ColorVisuals(mesh, vertex_colors=np.tile(
            np.round(np.asarray(color) * 255).astype(np.uint8), (len(vertices), 1)))
    elif mesh.visual.kind == "face":
        # Avoid optional SciPy face-to-vertex conversion in Trimesh's GLB
        # exporter, preserving distinct face colors through duplicated vertices.
        colors = np.repeat(mesh.visual.face_colors, 3, axis=0)
        mesh = trimesh.Trimesh(vertices=mesh.vertices[mesh.faces].reshape(-1, 3),
                               faces=np.arange(len(faces)*3).reshape(-1, 3),
                               vertex_colors=colors, process=False)
    environment.visual_meshes[name] = mesh


def compose_environment(reference="empty", *, base_xml=None, position=(0, 0, 0),
                        rpy=(0, 0, 0), scale=1):
    environment = read_environment(reference)
    root = ET.fromstring(base_xml) if base_xml is not None else ET.parse(ROOT / "mujoco/mearm.xml").getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", (ROOT / "models/mearm_v1/meshes").as_posix())
    compiler.set("texturedir", "")
    outer = transform_matrix(position, rpy, scale)
    # Plain support plane and solid background are shared by all default views.
    material = root.find("asset/material[@name='floor_mat']")
    if material is not None:
        material.attrib.pop("texture", None)
        material.set("rgba", _numbers(environment.definition.get("floor_rgba", [.88, .90, .92, 1])))
    for sky in list(root.findall("asset/texture[@type='skybox']")):
        root.find("asset").remove(sky)
    rgb = _numbers(environment.definition.get("background_rgb", [.92, .94, .96]))
    ET.SubElement(root.find("asset"), "texture", name="environment_sky", type="skybox",
                  builtin="flat", rgb1=rgb, rgb2=rgb, width="16", height="16")
    map_element = root.find("visual/map")
    if map_element is None:
        map_element = ET.SubElement(root.find("visual"), "map")
    map_element.set("zfar", "500")
    seen = set()
    common_fields = {"name", "type", "pos_m", "rpy_deg", "scale", "rgba", "collidable"}
    for index, item in enumerate(environment.definition["objects"]):
        if not isinstance(item, dict):
            raise ValueError("Each scene object must be an object.")
        name = item.get("name", f"object_{index}")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", name) or name in seen:
            raise ValueError("Scene object names must be unique letters, digits, '-' or '_'.")
        seen.add(name)
        kind = item.get("type")
        specific = {"mesh": {"file"}, "image": {"file", "width_m", "height_m"},
                    "box": {"size_m"}, "sphere": {"radius_m"}, "cylinder": {"radius_m", "height_m"}}
        if kind not in specific:
            raise ValueError(f"Unknown scene object type: {kind}")
        if set(item) - common_fields - specific[kind]:
            raise ValueError(f"Unknown fields on scene object {name}.")
        collidable = item.get("collidable", False)
        if not isinstance(collidable, bool):
            raise ValueError("collidable must be true or false.")
        color = _rgba(item["rgba"], name + " RGBA") if "rgba" in item else None
        pose = outer @ transform_matrix(item.get("pos_m", [0, 0, 0]),
                                        item.get("rpy_deg", [0, 0, 0]), item.get("scale", 1))
        stem = "environment_" + name
        if kind in ("mesh", "image"):
            if collidable:
                raise ValueError("Imported mesh/image backgrounds are visual-only; add collidable box proxies instead.")
            if not isinstance(item.get("file"), str):
                raise ValueError(f"Scene object {name} needs a file path.")
            path = (environment.directory / item["file"]).expanduser().resolve()
            if not path.is_file():
                raise ValueError(f"Scene asset not found: {path}")
            if kind == "mesh":
                meshes = _load_meshes(path)
            else:
                with Image.open(path) as image_file:
                    image = image_file.convert("RGB")
                width = float(item.get("width_m", 1))
                height = float(item.get("height_m", width * image.height / image.width))
                vector([width, height], 2, "Image dimensions", positive=True)
                # Images lie in local XY. The convenient direct-file default
                # is an upright backdrop behind the arm, not a camera overlay.
                if "pos_m" not in item and "rpy_deg" not in item:
                    pose = outer @ transform_matrix([.35, 0, .25], [90, 0, -90], item.get("scale", 1))
                mesh = trimesh.Trimesh(vertices=[[-width/2, -height/2, 0], [width/2, -height/2, 0],
                                                [width/2, height/2, 0], [-width/2, height/2, 0]],
                                       faces=[[0, 1, 2], [0, 2, 3]], process=False)
                mesh.visual = trimesh.visual.texture.TextureVisuals(
                    uv=[[0, 0], [1, 0], [1, 1], [0, 1]],
                    material=trimesh.visual.material.PBRMaterial(
                        baseColorTexture=image, baseColorFactor=[255, 255, 255, 255], doubleSided=True))
                meshes = [mesh]
            for part, mesh in enumerate(meshes):
                mesh.apply_transform(pose)
                _add_mesh(root, environment, f"{stem}_{part}", mesh, color)
        else:
            # Primitive transformations can contain only orthogonal scale axes;
            # reject shear rather than distort physical collision dimensions.
            scales = np.linalg.norm(pose[:3, :3], axis=0)
            rotation = pose[:3, :3] / scales
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
                raise ValueError("Rotated primitives with non-uniform outer scale create shear; use uniform scale.")
            if kind == "box":
                size = vector(item.get("size_m"), 3, "Box full dimensions", positive=True) * scales / 2
            elif kind == "sphere":
                radius = vector([item.get("radius_m")], 1, "Sphere radius", positive=True)[0]
                if not np.allclose(scales, scales[0]):
                    raise ValueError("Spheres require uniform scale.")
                size = [radius * scales[0]]
            else:
                radius, height = vector([item.get("radius_m"), item.get("height_m")], 2,
                                        "Cylinder radius/height", positive=True)
                if not math.isclose(scales[0], scales[1]):
                    raise ValueError("Cylinder X/Y scales must match.")
                size = [radius * scales[0], height * scales[2] / 2]
            matrix = np.eye(4)
            matrix[:3, :3] = rotation
            quat = trimesh.transformations.quaternion_from_matrix(matrix)
            ET.SubElement(root.find("worldbody"), "geom", name=stem, type=kind, size=_numbers(size),
                          pos=_numbers(pose[:3, 3]), quat=_numbers(quat),
                          rgba=_numbers(color if color is not None else [.65, .68, .72, 1]),
                          contype="1" if collidable else "0", conaffinity="3" if collidable else "0", group="1")
    return ET.tostring(root, encoding="unicode"), environment


def load_environment(reference="empty", **kwargs):
    xml, environment = compose_environment(reference, **kwargs)
    try:
        model = mujoco.MjModel.from_xml_string(xml, assets=environment.assets)
    except ValueError as exc:
        raise ValueError(f"Cannot compile scene '{environment.name}': {exc}") from exc
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, environment


def add_scene_arguments(parser):
    parser.add_argument("--scene", default="empty", help="empty (default), workshop, classroom, JSON, mesh or image path")
    parser.add_argument("--scene-scale", type=float, default=1, help="Environment unit conversion, e.g. .001 for mm meshes")
    parser.add_argument("--scene-pos", type=float, nargs=3, default=(0, 0, 0), metavar=("X", "Y", "Z"), help="Environment offset in metres")
    parser.add_argument("--scene-rpy", type=float, nargs=3, default=(0, 0, 0), metavar=("ROLL", "PITCH", "YAW"), help="Environment rotation in degrees")


def scene_kwargs(args):
    return {"position": args.scene_pos, "rpy": args.scene_rpy, "scale": args.scene_scale}
