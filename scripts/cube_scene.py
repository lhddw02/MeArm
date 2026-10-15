"""Build the existing MeArm model with four physical 20 mm coloured cubes.

The 80 mm platform raises the cubes into the Classic V1 arm's workspace.
Dimensions in this module use metres unless their name ends in ``_mm``.
The source arm XML, joints, actuators and passive couplings are preserved.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

if __package__:
    from ._runtime import configure_dependencies
else:
    from _runtime import configure_dependencies

ROOT = Path(__file__).resolve().parents[1]
configure_dependencies(ROOT)
import mujoco


CUBE_SIZE_M = 0.02
CUBE_MASS_KG = 0.008  # Estimate: a solid cube with density 1000 kg/m^3.
PLATFORM_TOP_M = 0.08
CUBES = [
    {"name": "red", "color": [0.9, 0.08, 0.07, 1.0], "position_mm": [130, -45, 90]},
    {"name": "green", "color": [0.1, 0.7, 0.2, 1.0], "position_mm": [180, 0, 90]},
    {"name": "blue", "color": [0.08, 0.3, 0.95, 1.0], "position_mm": [130, 45, 90]},
    {"name": "yellow", "color": [1.0, 0.8, 0.06, 1.0], "position_mm": [180, 45, 90]},
]
DEFAULT_TARGET_MM = (160, 0, 120)
GRASP_OPEN_RAD = .7


def _add_grip_pads(scene):
    """Scene-only wider jaw travel and visible soft pads for 20 mm cubes."""
    scene.find(".//joint[@name='left_finger_joint']").set('range','0 .7')
    scene.find(".//joint[@name='right_finger_joint']").set('range','-.7 0')
    scene.find("actuator/position[@name='gripper']").set('ctrlrange','0 .7')
    for side,sign in (('left',-1),('right',1)):
        body=scene.find(f".//body[@name='{side}_finger']")
        pad=body.find(f"geom[@name='{side}_finger_collision_0']")
        pad.attrib.update(size='.014 .0015 .010',
                          quat=f'.9747941071 0 0 {sign*.2231063621:.10f}',
                          friction='2 .02 .002',condim='6',
                          solref='.004 1',solimp='.999 .999 .001')
        # Visible pads exactly match the contact geometry; the acrylic STL stays.
        visual=dict(pad.attrib)
        visual.update(name=f'{side}_rubber_pad',contype='0',conaffinity='0',
                      group='2',rgba='.12 .14 .16 1')
        ET.SubElement(body,'geom',**visual)


def _numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def _target_metres(target_mm):
    if target_mm is None:
        target_mm = DEFAULT_TARGET_MM
    try:
        values = tuple(float(value) for value in target_mm)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Target position must contain three finite numbers in mm.") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("Target position must contain three finite numbers in mm.")
    return tuple(value / 1000 for value in values)


def build_scene_xml(target_mm=None,*,grasp_ready=False):
    """Return complete MJCF with absolute mesh paths for in-memory loading.

    ``target_mm`` controls the non-colliding magenta ``reach_target`` site;
    it does not move the arm. The controller applies position commands.
    """
    target = _target_metres(target_mm)
    tree = ET.parse(ROOT / "mujoco/mearm.xml")
    scene = tree.getroot()
    scene.set("model", "mearm_v1_cubes")
    scene.find("compiler").set("meshdir", (ROOT / "models/mearm_v1/meshes").as_posix())
    scene.find("statistic").attrib.update(center=".07 0 .08", extent=".40")
    if grasp_ready:
        _add_grip_pads(scene)
    world = scene.find("worldbody")

    platform = ET.SubElement(world, "body", name="cube_platform", pos=".16 0 0")
    common = dict(type="box", contype="1", conaffinity="3", friction=".8 .01 .001")
    ET.SubElement(platform, "geom", name="platform_top", size=".08 .08 .004",
                  pos="0 0 .076", rgba=".55 .58 .62 1", **common)
    # The tabletop spans x=80..240 mm, y=-80..80 mm; its upper face is z=80 mm.
    for index, (x, y) in enumerate(((-.065, -.065), (-.065, .065),
                                   (.065, -.065), (.065, .065))):
        ET.SubElement(platform, "geom", name=f"platform_leg_{index + 1}",
                      size=".006 .006 .036", pos=_numbers((x, y, .036)),
                      rgba=".35 .38 .42 1", **common)

    half_size = CUBE_SIZE_M / 2
    inertia = CUBE_MASS_KG * CUBE_SIZE_M ** 2 / 6
    for cube in CUBES:
        name = "cube_" + cube["name"]
        body = ET.SubElement(world, "body", name=name,
                             pos=_numbers(value / 1000 for value in cube["position_mm"]))
        ET.SubElement(body, "freejoint", name=name + "_joint")
        # The source arm disables inertia inference, so define the cube inertia.
        ET.SubElement(body, "inertial", pos="0 0 0", mass=f"{CUBE_MASS_KG:.12g}",
                      diaginertia=_numbers((inertia, inertia, inertia)))
        ET.SubElement(body, "geom", name=name + "_geom", type="box",
                      size=_numbers((half_size, half_size, half_size)),
                      rgba=_numbers(cube["color"]), contype="1", conaffinity="3",
                      friction="1 .01 .001", condim="4", solref=".006 1")
        if grasp_ready:
            body.find('geom').attrib.update(friction='2 .02 .002',condim='6',
                                           solref='.004 1',solimp='.999 .999 .001')

    ET.SubElement(world, "site", name="reach_target", type="sphere",
                  pos=_numbers(target), size=".004", rgba=".9 .1 .8 .55", group="1")
    if grasp_ready:
        ET.SubElement(world,'site',name='place_target',type='box',
                      pos='.18 -.045 .0805',size='.011 .011 .0005',
                      rgba='.1 .9 .6 0',group='1')
    ET.indent(tree, space="  ")
    return ET.tostring(scene, encoding="unicode", xml_declaration=True) + "\n"


def load_scene(target_mm=None,*,grasp_ready=False):
    """Compile the cube scene and return a model plus fresh, forwarded data."""
    model = mujoco.MjModel.from_xml_string(build_scene_xml(target_mm,grasp_ready=grasp_ready))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def write_scene(path=ROOT / "mujoco/mearm_cubes.xml",*,grasp_ready=False):
    """Save portable MJCF, resolving the mesh directory relative to its file."""
    path = Path(path).resolve()
    scene = ET.fromstring(build_scene_xml(grasp_ready=grasp_ready))
    meshdir = os.path.relpath(ROOT / "models/mearm_v1/meshes", path.parent)
    scene.find("compiler").set("meshdir", Path(meshdir).as_posix())
    tree = ET.ElementTree(scene)
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


if __name__ == "__main__":
    print(write_scene(grasp_ready=True))
