#!/usr/bin/env python3
"""Generate the Gazebo Sim model from the shared SI model specification."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = ROOT / "config" / "model_spec.json"
ENGINE = "gz-physics-bullet-featherstone-plugin"


def numbers(values: list[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def element(parent: ET.Element, tag: str, text=None, **attributes) -> ET.Element:
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def write_xml(path: Path, root: ET.Element) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def add_material(visual: ET.Element, color: list[float]) -> None:
    material = element(visual, "material")
    element(material, "ambient", numbers(color))
    element(material, "diffuse", numbers(color))
    element(material, "specular", "0.2 0.2 0.2 1")


def mesh_uri(model_name: str, mesh: str) -> str:
    """Accept both root-relative and model-relative specification mesh paths."""
    parts = list(Path(mesh).as_posix().split("/"))
    if "meshes" not in parts:
        raise ValueError(f"Mesh path has no meshes directory: {mesh}")
    relative = "/".join(parts[parts.index("meshes"):])
    return f"model://{model_name}/{relative}"


def build_model(spec: dict) -> ET.Element:
    root = ET.Element("sdf", version="1.11")
    model = element(root, "model", name=spec["name"])
    # Mimic constraints encode the two parallelogram compensations and the jaws.
    # Gazebo Harmonic requires Bullet Featherstone for these constraints.
    model.append(ET.Comment("Reduced MeArm mechanism; native mimic requires Bullet Featherstone."))
    element(model, "self_collide", "false")
    element(model, "static", "false")

    link_names = {link["name"] for link in spec["links"]}
    roots = [link for link in spec["links"] if link.get("parent") is None]
    if len(roots) != 1:
        raise ValueError("The model must contain exactly one root link")
    for link in spec["links"]:
        parent_name = link.get("parent")
        if parent_name is not None and parent_name not in link_names:
            raise ValueError(f"Unknown parent link {parent_name}")
        node = element(model, "link", name=link["name"])
        element(node, "pose", numbers(link["pos"] + link["rpy"]),
                relative_to=parent_name or "__model__")
        inertial = element(node, "inertial")
        element(inertial, "pose", numbers(link["com"] + [0, 0, 0]))
        element(inertial, "mass", f"{link['mass']:.12g}")
        inertia = element(inertial, "inertia")
        for tag, value in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), link["inertia"]):
            element(inertia, tag, f"{value:.12g}")
        if link.get("mesh"):
            visual = element(node, "visual", name=f"{link['name']}_visual")
            mesh = element(element(visual, "geometry"), "mesh")
            element(mesh, "uri", mesh_uri(spec["name"], link["mesh"]))
            element(mesh, "scale", "1 1 1")
            add_material(visual, link.get("color", [0.2, 0.6, 0.8, 1]))
        for index, collision in enumerate(link.get("collisions", [])):
            if collision["type"] != "box":
                raise ValueError(f"Unsupported collision type: {collision['type']}")
            shape = element(node, "collision", name=f"{link['name']}_collision_{index}")
            element(shape, "pose", numbers(collision.get("pos", [0, 0, 0]) +
                                            collision.get("rpy", [0, 0, 0])))
            box = element(element(shape, "geometry"), "box")
            element(box, "size", numbers(collision["size"]))
            surface = element(shape, "surface")
            friction = element(element(surface, "friction"), "ode")
            element(friction, "mu", "0.8")
            element(friction, "mu2", "0.8")

    tool = spec.get("tool")
    if tool is not None:
        frame = element(model, "frame", name="tool", attached_to=tool["link"])
        element(frame, "pose", numbers(tool["pos"] + [0, 0, 0]), relative_to=tool["link"])
    anchor = element(model, "joint", name="world_fixed", type="fixed")
    element(anchor, "parent", "world")
    element(anchor, "child", roots[0]["name"])
    for link in spec["links"]:
        joint = link.get("joint")
        if joint is None:
            if link.get("parent") is not None:
                fixed = element(model, "joint", name=f"{link['name']}_fixed", type="fixed")
                element(fixed, "parent", link["parent"])
                element(fixed, "child", link["name"])
            continue
        node = element(model, "joint", name=joint["name"], type="revolute")
        element(node, "parent", link["parent"])
        element(node, "child", link["name"])
        element(node, "pose", "0 0 0 0 0 0", relative_to=link["name"])
        axis = element(node, "axis")
        element(axis, "xyz", numbers(joint["axis"]), expressed_in=link["name"])
        limit = element(axis, "limit")
        for tag in ("lower", "upper", "effort", "velocity"):
            element(limit, tag, f"{joint[tag]:.12g}")
        dynamics = element(axis, "dynamics")
        element(dynamics, "damping", "0.005")
        if joint.get("mimic"):
            mimic_spec = joint["mimic"]
            mimic = element(axis, "mimic", joint=mimic_spec["joint"], axis="axis")
            element(mimic, "multiplier", f"{mimic_spec['multiplier']:.12g}")
            element(mimic, "offset", "0")
            element(mimic, "reference", "0")
        else:
            controller = element(model, "plugin",
                                 filename="gz-sim-joint-position-controller-system",
                                 name="gz::sim::systems::JointPositionController")
            element(controller, "joint_name", joint["name"])
            element(controller, "initial_position", "0")
            element(controller, "use_velocity_commands", "true")
            # In this controller mode cmd_max is the maximum joint speed.
            element(controller, "cmd_max", f"{min(joint['velocity'], 0.8):.12g}")
    element(model, "plugin", filename="gz-sim-joint-state-publisher-system",
            name="gz::sim::systems::JointStatePublisher")
    return root


def build_world(model_name: str) -> ET.Element:
    root = ET.Element("sdf", version="1.11")
    world = element(root, "world", name="mearm_demo")
    physics = element(world, "physics", name="1ms", type="ignored")
    element(physics, "max_step_size", "0.001")
    element(physics, "real_time_factor", "1")
    plugin = element(world, "plugin", filename="gz-sim-physics-system",
                     name="gz::sim::systems::Physics")
    element(element(plugin, "engine"), "filename", ENGINE)
    element(world, "plugin", filename="gz-sim-user-commands-system",
            name="gz::sim::systems::UserCommands")
    element(world, "plugin", filename="gz-sim-scene-broadcaster-system",
            name="gz::sim::systems::SceneBroadcaster")
    element(world, "gravity", "0 0 -9.81")
    scene = element(world, "scene")
    element(scene, "ambient", "0.55 0.55 0.55 1")
    element(scene, "background", "0.88 0.91 0.95 1")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 10 0 0 0")
    element(light, "diffuse", "0.9 0.9 0.9 1")
    element(light, "specular", "0.2 0.2 0.2 1")
    element(light, "direction", "-0.5 0.2 -1")
    element(light, "cast_shadows", "true")
    ground = element(world, "model", name="ground")
    element(ground, "static", "true")
    link = element(ground, "link", name="ground_link")
    for kind in ("collision", "visual"):
        shape = element(link, kind, name=f"ground_{kind}")
        geometry = element(shape, "geometry")
        plane = element(geometry, "plane")
        element(plane, "normal", "0 0 1")
        element(plane, "size", "2 2")
        if kind == "visual":
            add_material(shape, [0.72, 0.75, 0.8, 1])
    include = element(world, "include")
    element(include, "uri", f"model://{model_name}")
    element(include, "name", model_name)
    return root


def build(spec_path: Path = DEFAULT_SPEC) -> list[Path]:
    spec = json.loads(Path(spec_path).read_text(encoding="utf-8"))
    model_dir = ROOT / "models" / spec["name"]
    config = ET.Element("model")
    element(config, "name", "MeArm Classic V1 (reduced simulation model)")
    element(config, "version", "0.1.0")
    element(config, "sdf", "model.sdf", version="1.11")
    author = element(config, "author")
    element(author, "name", "MeArm simulation project")
    element(config, "description", "Parametric MeArm V1 model with native parallelogram mimic constraints. "
            "Requires Gazebo Sim with Bullet Featherstone. Approximate mass and collision properties.")
    return [write_xml(model_dir / "model.sdf", build_model(spec)),
            write_xml(model_dir / "model.config", config),
            write_xml(ROOT / "gazebo" / "world.sdf", build_world(spec["name"]))]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    args = parser.parse_args()
    for path in build(args.spec):
        print(path.relative_to(ROOT))


if __name__ == "__main__":
    main()
