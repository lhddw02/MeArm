"""Validate generated MeArm assets against independent geometry and MuJoCo.

Run with the bundled Python, or an environment containing requirements.txt.
Conda/venv packages take priority; compatible .deps wheels are a plain-Python fallback.
This is a headless test: it does not require an OpenGL display.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from _runtime import configure_dependencies
configure_dependencies(ROOT)

import mujoco
import numpy as np
import trimesh


INDEPENDENT = ("yaw_joint", "shoulder_joint", "elbow_joint", "left_finger_joint")
# Deliberately independent of the generated model's mimic declarations.
COUPLINGS = {
    "elbow_comp_joint": ("shoulder_joint", -1.0),
    "wrist_joint": ("elbow_joint", -1.0),
    "right_finger_joint": ("left_finger_joint", -1.0),
    "upper_parallel_joint": ("shoulder_joint", 1.0),
    "forearm_parallel_joint": ("elbow_joint", 1.0),
}


def analytic_tcp(values, cfg):
    """Expected FK from the official absolute-elevation convention, in SI."""
    yaw, shoulder_delta, beta, _ = values
    alpha = math.pi / 2 + shoulder_delta
    radius = (cfg["shoulder_offset_m"] + cfg["upper_arm_m"] * math.cos(alpha)
              + cfg["forearm_m"] * math.cos(beta) + cfg["wrist_to_tool_m"])
    height = (cfg["shoulder_height_m"] + cfg["upper_arm_m"] * math.sin(alpha)
              + cfg["forearm_m"] * math.sin(beta))
    return np.array([radius * math.cos(yaw), radius * math.sin(yaw), height])


def run_validation():
    spec = json.loads((ROOT / "config/model_spec.json").read_text())
    cfg = spec["config"]
    xml = ROOT / "mujoco/mearm.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "model_sha256": hashlib.sha256(xml.read_bytes()).hexdigest(),
        "mujoco_version": mujoco.__version__,
        "trimesh_version": trimesh.__version__,
        "seed": 20261015,
        "scope": "Ideal coupled V1 mechanism; estimated dynamics, not hardware identification.",
        "checks": {},
    }
    failures = []

    def check(name, ok, **metrics):
        report["checks"][name] = {"passed": bool(ok), **metrics}
        if not ok:
            failures.append(name)

    qadr = {name: int(model.joint(name).qposadr[0]) for name in (*INDEPENDENT, *COUPLINGS)}
    tcp_id = int(model.site("tcp").id)
    hand_id = int(model.body("hand").id)
    left_id = int(model.body("left_finger").id)
    right_id = int(model.body("right_finger").id)

    def set_state(values):
        mujoco.mj_resetData(model, data)
        for name, value in zip(INDEPENDENT, values):
            data.qpos[qadr[name]] = value
        for name, (source, multiplier) in COUPLINGS.items():
            data.qpos[qadr[name]] = multiplier * data.qpos[qadr[source]]
        data.ctrl[:] = values
        mujoco.mj_forward(model, data)

    def hand_tilt():
        normal = data.xmat[hand_id].reshape(3, 3)[:, 2]
        return math.acos(float(np.clip(normal[2], -1, 1)))

    def coupling_errors():
        return [abs(float(data.qpos[qadr[name]] - multiplier * data.qpos[qadr[source]]))
                for name, (source, multiplier) in COUPLINGS.items()]

    # Directly evaluate geometry over the full joint calibration envelope.
    rng = np.random.default_rng(report["seed"])
    ranges = np.array([model.jnt_range[int(model.joint(n).id)] for n in INDEPENDENT])
    states = rng.uniform(ranges[:, 0], ranges[:, 1], size=(100, 4))
    errors, tilts, jaw_errors = [], [], []
    for values in states:
        set_state(values)
        errors.append(float(np.linalg.norm(data.site_xpos[tcp_id] - analytic_tcp(values, cfg))))
        tilts.append(hand_tilt())
        # Compare jaw collision pad centers in the hand frame: the two pads
        # must stay mirror images and their gap must increase with opening.
        pads = []
        for bid, offset in ((left_id, [0.021, -0.007, -0.002]),
                            (right_id, [0.021, 0.007, -0.002])):
            world_pad = data.xpos[bid] + data.xmat[bid].reshape(3, 3) @ offset
            pads.append(data.xmat[hand_id].reshape(3, 3).T @ (world_pad - data.xpos[hand_id]))
        reflected = pads[1] * np.array([1, -1, 1])
        angle = values[3]
        expected_gap = 2 * (0.009 + 0.021 * math.sin(angle) - 0.007 * math.cos(angle))
        jaw_errors.append(max(float(np.linalg.norm(pads[0] - reflected)),
                              abs(float(pads[0][1] - pads[1][1]) - expected_gap)))
    check("random_kinematics", max(errors) < 1e-8, samples=len(states),
          max_tcp_error_m=max(errors), tolerance_m=1e-8)
    check("level_hand", max(tilts) < 1e-7, max_tilt_rad=max(tilts), tolerance_rad=1e-7)
    check("mirrored_jaws", max(jaw_errors) < 1e-8,
          max_pad_mirror_or_gap_error_m=max(jaw_errors), tolerance_m=1e-8)

    # Negative control: omitting shoulder compensation must be detectable.
    # This guards against a test oracle that would accept a serial-arm model
    # whose elbow and wrist accidentally inherit shoulder rotation.
    probe = np.array([0.31, 0.4, -0.2, 0.2])
    set_state(probe)
    data.qpos[qadr["elbow_comp_joint"]] = 0
    mujoco.mj_forward(model, data)
    bad_tcp_error = float(np.linalg.norm(data.site_xpos[tcp_id] - analytic_tcp(probe, cfg)))
    bad_hand_tilt = hand_tilt()
    check("compensation_negative_control", bad_tcp_error > 0.01 and bad_hand_tilt > 0.1,
          omitted_compensation_tcp_error_m=bad_tcp_error,
          omitted_compensation_hand_tilt_rad=bad_hand_tilt)

    # The default visual meshes contain disconnected plates. Disconnected is
    # valid; each closed surface still needs consistent winding and volume.
    mesh_results = []
    for link in spec["links"]:
        if not link["mesh"]:
            continue
        mesh = trimesh.load_mesh(ROOT / link["mesh"], process=True)
        valid = bool(mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0)
        mesh_results.append({"link": link["name"], "passed": valid,
                             "watertight": bool(mesh.is_watertight),
                             "consistent_winding": bool(mesh.is_winding_consistent),
                             "volume_m3": float(mesh.volume), "faces": int(len(mesh.faces))})
    check("visual_meshes", all(r["passed"] for r in mesh_results), meshes=mesh_results)

    inertia_results = []
    for link in spec["links"]:
        xx, yy, zz, xy, xz, yz = link["inertia"]
        eigenvalues = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        valid = (link["mass"] > 0 and np.all(eigenvalues > 0)
                 and eigenvalues[-1] <= eigenvalues[0] + eigenvalues[1] + 1e-12)
        inertia_results.append({"link": link["name"], "passed": bool(valid),
                                "mass_kg": link["mass"],
                                "principal_moments_kg_m2": eigenvalues.tolist()})
    compiled_valid = bool(np.all(model.body_mass[1:] > 0) and np.all(model.body_inertia[1:] > 0))
    check("physical_inertias", all(r["passed"] for r in inertia_results) and compiled_valid,
          compiled_positive=compiled_valid, links=inertia_results)

    expected_actuators = ("base", "shoulder", "elbow", "gripper")
    actuator_valid = model.nu == 4
    for aid, (name, joint) in enumerate(zip(expected_actuators, INDEPENDENT)):
        actuator_valid &= (model.actuator(aid).name == name
                           and int(model.actuator_trnid[aid, 0]) == int(model.joint(joint).id)
                           and bool(model.actuator_forcelimited[aid]))
    check("actuator_bindings", actuator_valid, independent_joints=list(INDEPENDENT))

    # Exercise torque saturation deliberately. Merely observing a gentle
    # trajectory below the configured torque would not verify clamping.
    saturated_forces = []
    configured_force_limits = np.max(np.abs(model.actuator_forcerange), axis=1)
    for start, target in ((ranges[:, 0], ranges[:, 1]), (ranges[:, 1], ranges[:, 0])):
        set_state(start)
        data.ctrl[:] = target
        mujoco.mj_forward(model, data)
        saturated_forces.append(data.actuator_force.copy())
    saturated_forces = np.array(saturated_forces)
    saturation_valid = (np.allclose(saturated_forces[0], configured_force_limits, atol=1e-10)
                        and np.allclose(saturated_forces[1], -configured_force_limits, atol=1e-10))
    check("actuator_saturation", saturation_valid,
          positive_step_force_nm=saturated_forces[0].tolist(),
          negative_step_force_nm=saturated_forces[1].tolist(),
          expected_magnitude_nm=configured_force_limits.tolist())

    # An unloaded, slow sweep is above the floor throughout. This exercises
    # equality constraints, gravity, position control, force clamping, and
    # stability together, without a renderer or kinematically teleporting joints.
    initial = np.array([0.0, 0.0, 0.0, 0.2])
    set_state(initial)
    dt = float(model.opt.timestep)
    duration = 12.0
    max_coupling = np.zeros(len(COUPLINGS))
    max_force = np.zeros(model.nu)
    max_velocity = 0.0
    max_qpos = 0.0
    max_dynamic_tilt = 0.0
    max_tracking = np.zeros(4)
    max_tcp_tracking = 0.0
    min_tcp_height = float("inf")
    max_contacts = 0
    finite = True
    final_tracking = np.zeros(4)
    for step in range(round(duration / dt)):
        t = step * dt
        if 2.0 <= t < 10.0:
            phase = 2 * math.pi * (t - 2.0) / 8.0
            desired = np.array([0.4 * math.sin(phase), 0.35 * math.sin(phase),
                                0.3 * math.sin(phase), 0.2 + 0.15 * math.sin(phase)])
        else:
            desired = initial
        data.ctrl[:] = desired
        mujoco.mj_step(model, data)
        values = np.array([data.qpos[qadr[n]] for n in INDEPENDENT])
        finite &= bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
        max_coupling = np.maximum(max_coupling, coupling_errors())
        max_force = np.maximum(max_force, np.abs(data.actuator_force))
        max_velocity = max(max_velocity, float(np.max(np.abs(data.qvel))))
        max_qpos = max(max_qpos, float(np.max(np.abs(data.qpos))))
        max_dynamic_tilt = max(max_dynamic_tilt, hand_tilt())
        max_contacts = max(max_contacts, int(data.ncon))
        min_tcp_height = min(min_tcp_height, float(data.site_xpos[tcp_id, 2]))
        if t >= 2.0:
            max_tracking = np.maximum(max_tracking, np.abs(values - desired))
            max_tcp_tracking = max(max_tcp_tracking, float(np.linalg.norm(
                data.site_xpos[tcp_id] - analytic_tcp(desired, cfg))))
        final_tracking = np.abs(values - desired)
    warnings = {mujoco.mjtWarning(i).name: int(w.number)
                for i, w in enumerate(data.warning) if w.number}
    check("dynamic_stability", finite and max_qpos < 1.0 and max_velocity < cfg["servo_speed_rad_s"]
          and not warnings and max_contacts == 0,
          duration_s=duration, timestep_s=dt, steps=round(duration / dt),
          finite_state=finite, max_abs_qpos_rad=max_qpos,
          max_abs_velocity_rad_s=max_velocity, velocity_bound_rad_s=cfg["servo_speed_rad_s"],
          min_tcp_height_m=min_tcp_height, max_contacts=max_contacts, warnings=warnings)
    check("dynamic_couplings", float(max(max_coupling)) < 1e-3 and max_dynamic_tilt < 2e-3,
          max_error_rad=dict(zip(COUPLINGS, max_coupling.tolist())), tolerance_rad=1e-3,
          max_hand_tilt_rad=max_dynamic_tilt, hand_tilt_tolerance_rad=2e-3)
    limits = np.max(np.abs(model.actuator_forcerange), axis=1)
    check("actuator_force_limits", bool(np.all(max_force <= limits + 1e-10)),
          max_abs_force_nm=dict(zip(expected_actuators, max_force.tolist())),
          configured_limits_nm=limits.tolist())
    check("dynamic_tracking", float(max(max_tracking)) < 0.04 and float(max(final_tracking)) < 0.01,
          max_joint_error_rad=dict(zip(INDEPENDENT, max_tracking.tolist())),
          final_joint_error_rad=dict(zip(INDEPENDENT, final_tracking.tolist())),
          max_tcp_tracking_error_m=max_tcp_tracking,
          max_joint_error_tolerance_rad=0.04, final_error_tolerance_rad=0.01)

    report["passed"] = not failures
    report["failed_checks"] = failures
    output = ROOT / "docs/validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    result = run_validation()
    for name, value in result["checks"].items():
        print(f'{"PASS" if value["passed"] else "FAIL"}: {name}')
    print(f'Report: {ROOT / "docs/validation.json"}')
    sys.exit(0 if result["passed"] else 1)
