#!/usr/bin/env python3
"""Check local Gazebo assets and mechanism invariants without simulator libraries.

The optional official schema subset check validates element names, attributes,
value types and cardinalities. It complements, and does not replace, `gz sdf -k`.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_DIR = ROOT / "references" / "sdformat-1.11"
SCHEMA_URL = "https://raw.githubusercontent.com/gazebosim/sdformat/sdf14/sdf/1.11/"
ENGINE = "gz-physics-bullet-featherstone-plugin"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def vector(text: str | None, size: int) -> list[float]:
    values = [float(value) for value in (text or "").split()]
    require(len(values) == size and all(math.isfinite(value) for value in values),
            f"Expected {size} finite numbers, got {text!r}")
    return values


def close(actual, expected, label: str, *, absolute=1e-10) -> None:
    require(len(actual) == len(expected) and all(
        math.isclose(a, b, rel_tol=1e-9, abs_tol=absolute) for a, b in zip(actual, expected)),
        f"{label}: {actual!r} does not match {expected!r}")


class SchemaCheck:
    """Read the native SDFormat description schema, restricted to used elements."""

    def __init__(self, directory: Path, fetch: bool):
        self.directory = directory
        self.fetch = fetch
        self.cache: dict[str, ET.Element] = {}

    def load(self, filename: str) -> ET.Element:
        require(Path(filename).name == filename, "Unexpected schema filename")
        if filename not in self.cache:
            path = self.directory / filename
            if not path.exists() and self.fetch:
                path.parent.mkdir(parents=True, exist_ok=True)
                with urllib.request.urlopen(SCHEMA_URL + filename, timeout=30) as response:
                    path.write_bytes(response.read())
            self.cache[filename] = ET.parse(path).getroot()
        return self.cache[filename]

    def typed(self, text: str, kind: str, path: str) -> None:
        text = text.strip()
        if kind == "string":
            return
        if kind == "bool":
            require(text in ("true", "false", "0", "1"), f"{path}: invalid bool {text!r}")
        elif kind in ("double", "float"):
            require(math.isfinite(float(text)), f"{path}: nonfinite number")
        elif kind in ("int", "unsigned int", "unsigned long", "uint64_t"):
            value = int(text)
            require(not kind.startswith("unsigned") or value >= 0, f"{path}: negative unsigned value")
        elif kind in ("vector2", "vector2d", "vector3", "vector3d", "color", "pose"):
            vector(text, {"vector2": 2, "vector2d": 2, "vector3": 3, "vector3d": 3,
                          "color": 4, "pose": 6}[kind])
        else:
            raise ValueError(f"{path}: unimplemented schema type {kind!r}")

    def validate(self, node: ET.Element, schema: ET.Element | None = None, path="") -> None:
        schema = self.load("root.sdf") if schema is None else schema
        path += "/" + node.tag
        require(node.tag == schema.get("name"), f"{path}: schema name mismatch")
        attributes = {item.get("name"): item for item in schema.findall("attribute")}
        require(set(node.attrib) <= set(attributes),
                f"{path}: unknown attributes {set(node.attrib) - set(attributes)}")
        for name, description in attributes.items():
            if description.get("required") == "1":
                require(name in node.attrib, f"{path}: missing required attribute {name}")
            if name in node.attrib:
                self.typed(node.attrib[name], description.get("type", "string"), path + "@" + name)
        if schema.get("type"):
            self.typed(node.text or "", schema.get("type"), path)
        if schema.get("copy_data") == "true" or schema.find("element[@copy_data='true']") is not None:
            return  # Plugin content is intentionally arbitrary in SDFormat.
        child_schemas = {child.get("name"): child for child in schema.findall("element")}
        # Geometry files use e.g. mesh_shape.sdf for the <mesh> element.
        for include in schema.findall("include"):
            name = Path(include.get("filename")).stem.removesuffix("_shape")
            if any(child.tag == name for child in node):
                included = ET.fromstring(ET.tostring(self.load(include.get("filename"))))
                included.set("required", include.get("required", included.get("required", "0")))
                child_schemas[included.get("name")] = included
        counts = Counter(child.tag for child in node)
        for name, description in child_schemas.items():
            required = description.get("required", "0")
            if required not in ("*", "+"):
                require(counts[name] <= 1, f"{path}: duplicate {name}")
            # libsdformat fills omitted scalar defaults, including required ones.
            if required in ("1", "+") and "default" not in description.attrib:
                require(counts[name] >= 1, f"{path}: missing required element {name}")
        for child in node:
            require(child.tag in child_schemas, f"{path}: unknown element {child.tag}")
            self.validate(child, child_schemas[child.tag], path)

    def manifest(self) -> None:
        files = {name: hashlib.sha256((self.directory / name).read_bytes()).hexdigest()
                 for name in sorted(self.cache)}
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / "manifest.json").write_text(json.dumps({
            "source": SCHEMA_URL,
            "scope": "Only native description schema files used by the generated model and world.",
            "sha256": files,
        }, indent=2) + "\n", encoding="utf-8")


def det(matrix) -> float:
    a, b, c = matrix
    return (a[0] * (b[1] * c[2] - b[2] * c[1]) -
            a[1] * (b[0] * c[2] - b[2] * c[0]) +
            a[2] * (b[0] * c[1] - b[1] * c[0]))


def check_inertia(link: ET.Element) -> None:
    name = link.get("name")
    mass = float(link.findtext("inertial/mass"))
    require(math.isfinite(mass) and mass > 0, f"{name}: nonpositive mass")
    node = link.find("inertial/inertia")
    xx, yy, zz, xy, xz, yz = [float(node.findtext(key)) for key in
                              ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
    require(all(math.isfinite(value) for value in (xx, yy, zz, xy, xz, yz)),
            f"{name}: nonfinite inertia")
    inertia = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    require(xx > 0 and xx * yy - xy * xy > 0 and det(inertia) > 0,
            f"{name}: inertia matrix is not positive definite")
    # A physical inertia also requires the triangle inequalities on principal
    # moments. This is equivalent to positive semidefinite second moments C.
    half_trace = (xx + yy + zz) / 2
    second = [[(half_trace if i == j else 0) - inertia[i][j]
               for j in range(3)] for i in range(3)]
    scale = max(xx, yy, zz)
    require(all(second[i][i] >= -1e-9 * scale for i in range(3)) and
            all(second[i][i] * second[j][j] - second[i][j] ** 2 >= -1e-9 * scale ** 2
                for i, j in ((0, 1), (0, 2), (1, 2))) and
            det(second) >= -1e-9 * scale ** 3,
            f"{name}: inertia violates physical principal-moment triangle inequalities")


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(matrix, point):
    return [sum(matrix[i][j] * point[j] for j in range(3)) for i in range(3)]


def axis_rotation(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]


def rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    return multiply(axis_rotation([0, 0, 1], yaw),
                    multiply(axis_rotation([0, 1, 0], pitch), axis_rotation([1, 0, 0], roll)))


def check_kinematics(links, joints, spec) -> int:
    """Forward-kinematic checks evaluate the generated SDF, not builder helpers."""
    by_child = {joint.findtext("child"): joint for joint in joints.values()}
    samples = [(0, 0, 0, 0), (0.4, -0.3, 0.25, 0.3),
               (-0.3, 0.5, -0.4, 0.1), (0.7, -0.6, 0.6, 0.5)]
    for yaw, shoulder, forearm, grip in samples:
        coordinates = {"yaw_joint": yaw, "shoulder_joint": shoulder,
                       "elbow_joint": forearm, "left_finger_joint": grip}

        def coordinate(name):
            if name not in coordinates:
                mimic = joints[name].find("axis/mimic")
                require(mimic is not None, f"No kinematic coordinate for {name}")
                coordinates[name] = float(mimic.findtext("multiplier")) * (
                    coordinate(mimic.get("joint")) - float(mimic.findtext("reference", "0"))) + float(
                        mimic.findtext("offset", "0"))
            return coordinates[name]

        transforms = {}

        def transform(name):
            if name in transforms:
                return transforms[name]
            link = links[name]
            pose = link.find("pose")
            values = vector(pose.text, 6)
            relative = pose.get("relative_to", "__model__")
            if relative == "__model__":
                parent_pos, parent_rot = [0, 0, 0], rpy_rotation([0, 0, 0])
            else:
                parent_pos, parent_rot = transform(relative)
            displacement = rotate(parent_rot, values[:3])
            position = [a + b for a, b in zip(parent_pos, displacement)]
            rotation = multiply(parent_rot, rpy_rotation(values[3:]))
            joint = by_child[name]
            if joint.get("type") == "revolute":
                local_axis = vector(joint.findtext("axis/xyz"), 3)
                rotation = multiply(rotation, axis_rotation(local_axis, coordinate(joint.get("name"))))
            transforms[name] = position, rotation
            return position, rotation

        for name in links:
            transform(name)
        close(rotate(transform("upper")[1], [1, 0, 0]),
              [-math.sin(shoulder) * math.cos(yaw), -math.sin(shoulder) * math.sin(yaw),
               math.cos(shoulder)], "upper elevation")
        close(rotate(transform("forearm")[1], [1, 0, 0]),
              [math.cos(forearm) * math.cos(yaw), math.cos(forearm) * math.sin(yaw),
               math.sin(forearm)], "forearm absolute elevation")
        close(rotate(transform("hand")[1], [0, 0, 1]), [0, 0, 1], "level gripper")
        close(rotate(transform("hand")[1], [1, 0, 0]),
              [math.cos(yaw), math.sin(yaw), 0], "gripper heading")
        close([coordinate("right_finger_joint")], [-grip], "mirrored jaws")
        hand_pos, hand_rot = transform(spec["tool"]["link"])
        tcp = [a + b for a, b in zip(hand_pos, rotate(hand_rot, spec["tool"]["pos"]))]
        config = spec["config"]
        radius = (config["shoulder_offset_m"] - config["upper_arm_m"] * math.sin(shoulder) +
                  config["forearm_m"] * math.cos(forearm) + config["wrist_to_tool_m"])
        expected = [radius * math.cos(yaw), radius * math.sin(yaw),
                    config["shoulder_height_m"] + config["upper_arm_m"] * math.cos(shoulder) +
                    config["forearm_m"] * math.sin(forearm)]
        close(tcp, expected, "TCP position")
    return len(samples)


def validate(spec_path: Path, schema_dir: Path = SCHEMA_DIR, fetch_schema=False, run_gz=False) -> dict:
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    model_path = ROOT / "models" / spec["name"] / "model.sdf"
    world_path = ROOT / "gazebo" / "world.sdf"
    model_document = ET.parse(model_path).getroot()
    world_document = ET.parse(world_path).getroot()
    require(model_document.get("version") == world_document.get("version") == "1.11", "Wrong SDF version")
    model = model_document.find("model")
    require(model.get("name") == spec["name"], "Wrong model name")
    tool_frame = model.find("frame[@name='tool']")
    require(tool_frame is not None and tool_frame.get("attached_to") == spec["tool"]["link"] and
            tool_frame.find("pose").get("relative_to") == spec["tool"]["link"], "Missing or invalid tool frame")
    close(vector(tool_frame.findtext("pose"), 6), spec["tool"]["pos"] + [0, 0, 0], "Tool frame pose")
    link_nodes, joint_nodes = model.findall("link"), model.findall("joint")
    links = {link.get("name"): link for link in link_nodes}
    joints = {joint.get("name"): joint for joint in joint_nodes}
    require(len(links) == len(link_nodes) and len(joints) == len(joint_nodes), "Duplicate names")
    require(set(links) == {link["name"] for link in spec["links"]}, "SDF/spec link names differ")
    by_child = {joint.findtext("child"): joint for joint in joints.values()}
    require(len(by_child) == len(joints) == len(links) and set(by_child) == set(links),
            "Every link must have exactly one incoming joint, including the world anchor")
    for joint in joints.values():
        require(joint.findtext("parent") in set(links) | {"world"}, "Invalid joint parent")
        require(joint.findtext("child") in links, "Invalid joint child")
    for name in links:
        visited = set()
        while name != "world":
            require(name not in visited, "Cyclic link graph")
            visited.add(name)
            name = by_child[name].findtext("parent")
    require(joints["world_fixed"].get("type") == "fixed" and
            joints["world_fixed"].findtext("parent") == "world", "Missing fixed world anchor")

    mesh_count, collision_count = 0, 0
    for description in spec["links"]:
        name = description["name"]
        link = links[name]
        check_inertia(link)
        close(vector(link.findtext("pose"), 6), description["pos"] + description["rpy"], f"{name} pose")
        require(link.find("pose").get("relative_to") == (description["parent"] or "__model__"),
                f"{name}: wrong pose frame")
        close([float(link.findtext("inertial/mass"))], [description["mass"]], f"{name} mass")
        close(vector(link.findtext("inertial/pose"), 6), description["com"] + [0, 0, 0], f"{name} COM")
        close([float(link.findtext("inertial/inertia/" + key)) for key in
               ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")], description["inertia"], f"{name} inertia",
              absolute=1e-16)
        for uri in link.findall("visual/geometry/mesh/uri"):
            parsed = urllib.parse.urlsplit(uri.text)
            require(parsed.scheme == "model" and parsed.netloc == spec["name"], "Wrong mesh URI scheme/model")
            target = (ROOT / "models" / parsed.netloc / parsed.path.lstrip("/")).resolve()
            require(target.is_relative_to((ROOT / "models" / spec["name"]).resolve()) and
                    target.is_file() and target.stat().st_size > 84, f"Missing/empty mesh {target}")
            mesh_count += 1
        collisions = link.findall("collision")
        require(len(collisions) == len(description["collisions"]), f"{name}: collision count differs")
        for collision, expected in zip(collisions, description["collisions"]):
            size = vector(collision.findtext("geometry/box/size"), 3)
            require(all(value > 0 for value in size), "Nonpositive collision size")
            close(size, expected["size"], f"{name} collision size")
            close(vector(collision.findtext("pose"), 6), expected["pos"] + expected["rpy"],
                  f"{name} collision pose")
            collision_count += 1
        joint_spec = description.get("joint")
        joint = by_child[name]
        require(joint.findtext("parent") == (description["parent"] or "world"), f"{name}: wrong joint parent")
        if joint_spec is None:
            require(joint.get("type") == "fixed", f"{name}: expected fixed joint")
            continue
        require(joint.get("name") == joint_spec["name"] and joint.get("type") == "revolute", f"{name}: wrong joint")
        require(joint.find("pose").get("relative_to") == name, f"{name}: wrong joint pose frame")
        close(vector(joint.findtext("pose"), 6), [0] * 6, f"{name} joint origin")
        axis = joint.find("axis/xyz")
        require(axis.get("expressed_in") == name, f"{name}: wrong axis frame")
        xyz = vector(axis.text, 3)
        close(xyz, joint_spec["axis"], f"{name} axis")
        close([sum(value*value for value in xyz)], [1], f"{name} unit axis")
        lower, upper, effort, velocity = [float(joint.findtext("axis/limit/" + key))
                                          for key in ("lower", "upper", "effort", "velocity")]
        require(lower <= 0 <= upper and lower < upper and effort > 0 and velocity > 0,
                f"{name}: invalid joint limits")
        close([lower, upper, effort, velocity], [joint_spec[key] for key in
                                               ("lower", "upper", "effort", "velocity")], f"{name} limits")
        mimic = joint.find("axis/mimic")
        require((mimic is None) == (joint_spec.get("mimic") is None), f"{name}: mimic mismatch")
        if mimic is not None:
            leader_name = mimic.get("joint")
            require(leader_name in joints and leader_name != joint.get("name") and
                    mimic.get("axis") == "axis", f"{name}: invalid mimic leader")
            require(leader_name == joint_spec["mimic"]["joint"], f"{name}: wrong mimic endpoint")
            multiplier = float(mimic.findtext("multiplier"))
            close([multiplier], [joint_spec["mimic"]["multiplier"]], f"{name} mimic ratio")
            reference, offset = float(mimic.findtext("reference", "0")), float(mimic.findtext("offset", "0"))
            mapped = [multiplier * (float(joints[leader_name].findtext("axis/limit/" + key)) - reference) + offset
                      for key in ("lower", "upper")]
            require(min(mapped) >= lower - 1e-10 and max(mapped) <= upper + 1e-10,
                    f"{name}: mimic and leader limits disagree")
            visited = {joint.get("name")}
            while leader_name:
                require(leader_name not in visited, "Cyclic mimic graph")
                visited.add(leader_name)
                next_mimic = joints[leader_name].find("axis/mimic")
                leader_name = next_mimic.get("joint") if next_mimic is not None else None

    controllers = model.findall("plugin[@filename='gz-sim-joint-position-controller-system']")
    actuated = {joint.get("name") for joint in joints.values()
                if joint.get("type") == "revolute" and joint.find("axis/mimic") is None}
    require({controller.findtext("joint_name") for controller in controllers} == actuated and
            len(controllers) == len(actuated) == 4, "Independent controller set is incorrect")
    for controller in controllers:
        require(controller.get("name") == "gz::sim::systems::JointPositionController" and
                controller.findtext("use_velocity_commands") == "true", "Wrong controller configuration")
        require(0 < float(controller.findtext("cmd_max")) <= float(
            joints[controller.findtext("joint_name")].findtext("axis/limit/velocity")), "Invalid controller speed")
    world = world_document.find("world")
    physics = world.find("plugin[@filename='gz-sim-physics-system']")
    require(physics is not None and physics.get("name") == "gz::sim::systems::Physics" and
            physics.findtext("engine/filename") == ENGINE, "Bullet Featherstone is not explicitly selected")
    require(world.findtext("include/uri") == f"model://{spec['name']}", "Wrong world model URI")
    require(float(world.findtext("physics/max_step_size")) <= 0.001, "World step is too large")
    config_path = model_path.with_name("model.config")
    model_config = ET.parse(config_path).getroot()
    require(model_config.findtext("sdf") == "model.sdf" and
            model_config.find("sdf").get("version") == "1.11", "Wrong model.config entry")
    samples = check_kinematics(links, joints, spec)
    schemas = SchemaCheck(schema_dir, fetch_schema)
    checked_schemas = 0
    if fetch_schema or (schema_dir / "root.sdf").exists():
        schemas.validate(model_document)
        schemas.validate(world_document)
        if fetch_schema:
            schemas.manifest()
        checked_schemas = len(schemas.cache)
    gz_path = shutil.which("gz")
    runtime = "not run: Gazebo is not installed" if gz_path is None else "not requested"
    if run_gz:
        require(gz_path is not None, "Cannot run Gazebo validation: gz is not on PATH")
        env = dict(os.environ)
        env["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "models") + os.pathsep + env.get("GZ_SIM_RESOURCE_PATH", "")
        for path in (model_path, world_path):
            subprocess.run([gz_path, "sdf", "-k", str(path)], check=True, env=env, timeout=30)
        runtime = "gz sdf -k passed; dynamics were not run"
    return {"links": len(links), "joints": len(joints), "meshes": mesh_count,
            "collision_proxies": collision_count,
            "mimic_constraints": sum(joint.find("axis/mimic") is not None for joint in joints.values()),
            "controllers": len(controllers), "kinematic_samples": samples,
            "official_schema_files_checked": checked_schemas, "gazebo_runtime": runtime}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=ROOT / "config" / "model_spec.json")
    parser.add_argument("--schema-dir", type=Path, default=SCHEMA_DIR)
    parser.add_argument("--fetch-schema", action="store_true", help="download only required official schema files")
    parser.add_argument("--run-gz", action="store_true", help="also run gz sdf -k when installed")
    parser.add_argument("--report", type=Path, help="also save the JSON summary to this path")
    args = parser.parse_args()
    try:
        report = json.dumps(validate(args.spec, args.schema_dir, args.fetch_schema, args.run_gz), indent=2) + "\n"
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(report, encoding="utf-8")
        print(report, end="")
    except (ValueError, OSError, ET.ParseError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"Validation failed: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
