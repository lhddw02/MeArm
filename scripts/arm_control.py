"""Joint and Cartesian position commands for the coupled Classic V1 MeArm.

User commands use degrees and millimetres. Shoulder and forearm angles are
absolute elevations; neutral is shoulder=90, forearm=0. Position actuator
targets use radians, with the shoulder measured relative to vertical.
The gripper stays level because MuJoCo solves the model's passive couplings.
"""
from __future__ import annotations

from collections.abc import Mapping
from functools import lru_cache
import json
import math
from pathlib import Path

if __package__:
    from ._runtime import configure_dependencies
else:
    from _runtime import configure_dependencies

ROOT = Path(__file__).resolve().parents[1]
configure_dependencies(ROOT)
import numpy as np


ACTUATOR_NAMES = ("base", "shoulder", "elbow", "gripper")
JOINT_LABELS = ("yaw", "shoulder", "forearm", "grip")
_GEOMETRY_KEYS = ("upper_arm_m", "forearm_m", "shoulder_offset_m",
                  "shoulder_height_m", "wrist_to_tool_m")


@lru_cache(maxsize=1)
def _default_config():
    return json.loads((ROOT / "config/mearm_v1.json").read_text(encoding="utf-8"))


def _geometry(config):
    if config is None:
        config = _default_config()
    if not isinstance(config, Mapping):
        raise ValueError("Geometry config must be a mapping of model dimensions.")
    if "config" in config:
        config = config["config"]
    try:
        lengths = tuple(float(config[key]) for key in _GEOMETRY_KEYS)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Geometry config is missing valid dimensions.") from exc
    if not all(math.isfinite(value) for value in lengths) or min(lengths[:2]) <= 0:
        raise ValueError("Arm lengths must be finite and positive; offsets must be finite.")
    return lengths


def _finite_vector(values, size, label):
    try:
        vector = np.asarray(values, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain finite numbers.") from exc
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must contain {size} finite numbers.")
    return vector.copy()


def forward_kinematics(angles_rad, config=None):
    """Return ideal TCP [X forward, Y left, Z up] in mm for three angles.

    ``angles_rad`` is [yaw, shoulder delta from vertical, forearm elevation].
    A fourth gripper angle is accepted and does not affect the TCP.
    """
    try:
        count = len(angles_rad)
    except TypeError as exc:
        raise ValueError("Joint angles must contain three or four finite numbers.") from exc
    if count not in (3, 4):
        raise ValueError("Joint angles must contain three or four finite numbers.")
    yaw, shoulder, beta = _finite_vector(angles_rad, count, "Joint angles")[:3]
    upper, forearm, offset, height, tool = _geometry(config)
    alpha = math.pi / 2 + shoulder
    radius = offset + upper * math.cos(alpha) + forearm * math.cos(beta) + tool
    z = height + upper * math.sin(alpha) + forearm * math.sin(beta)
    return 1000 * np.array([radius * math.cos(yaw), radius * math.sin(yaw), z])


def inverse_kinematics(x_mm, y_mm, z_mm, config=None):
    """Return both geometric yaw/shoulder/forearm solutions in radians.

    This pure helper checks the two-link reach triangle. MeArmController also
    checks the actuator limits before selecting and applying a solution.
    """
    x, y, z = _finite_vector((x_mm, y_mm, z_mm), 3, "TCP position") / 1000
    upper, forearm, offset, height, tool = _geometry(config)
    yaw = math.atan2(y, x)
    u = math.hypot(x, y) - offset - tool
    v = z - height
    cosine = (u * u + v * v - upper * upper - forearm * forearm) / (2 * upper * forearm)
    if cosine < -1 - 1e-12 or cosine > 1 + 1e-12:
        raise ValueError("TCP position is outside the arm's geometric reach.")
    delta_abs = math.acos(max(-1.0, min(1.0, cosine)))
    candidates = []
    for delta in (delta_abs, -delta_abs):
        alpha = math.atan2(v, u) - math.atan2(forearm * math.sin(delta),
                                             upper + forearm * math.cos(delta))
        beta = alpha + delta
        # Equivalent full turns must not make an otherwise valid branch fail
        # the finite stock servo limits around the neutral pose.
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        beta = math.atan2(math.sin(beta), math.cos(beta))
        candidates.append(np.array([yaw, alpha - math.pi / 2, beta]))
    return candidates


class MeArmController:
    """Write position servo targets without editing joint state or couplings.

    Targets are read from data.ctrl each time so native MuJoCo Control sliders
    and API/terminal commands can be used together. Call on the physics thread,
    and use the viewer lock when a passive viewer is running.
    """

    def __init__(self, model, data, config=None):
        self.model = model
        self.data = data
        self.config = dict(_default_config() if config is None else config)
        _geometry(self.config)
        try:
            self.actuator_ids = np.array([int(model.actuator(name).id)
                                          for name in ACTUATOR_NAMES], dtype=int)
            self._tcp_id = int(model.site("tcp").id)
        except (KeyError, ValueError) as exc:
            raise ValueError("Model needs base, shoulder, elbow, gripper actuators and a tcp site.") from exc
        self._limits = np.array(model.actuator_ctrlrange[self.actuator_ids], dtype=float, copy=True)
        if (not np.all(model.actuator_ctrllimited[self.actuator_ids])
                or not np.all(np.isfinite(self._limits))
                or np.any(self._limits[:, 0] > self._limits[:, 1])):
            raise ValueError("All MeArm actuators need finite position control limits.")
        self._validate(self.target)

    @property
    def target(self):
        """Current servo commands in radians, in canonical four-actuator order."""
        return np.array(self.data.ctrl[self.actuator_ids], dtype=float, copy=True)

    @property
    def target_degrees(self):
        degrees = np.rad2deg(self.target)
        degrees[1] += 90
        return degrees

    @property
    def limits_degrees(self):
        degrees = np.rad2deg(self._limits)
        degrees[1] += 90
        return degrees

    @property
    def tcp_position_mm(self):
        """Actual TCP from the latest MuJoCo forward/step calculation, in mm."""
        return 1000 * self.data.site_xpos[self._tcp_id].copy()

    def _validate(self, candidate):
        candidate = _finite_vector(candidate, 4, "Servo targets")
        for index, (value, (lower, upper)) in enumerate(zip(candidate, self._limits)):
            if value < lower - 1e-10 or value > upper + 1e-10:
                offset = 90 if index == 1 else 0
                raise ValueError(f"{JOINT_LABELS[index]} target {math.degrees(value) + offset:.3f} deg "
                                 f"is outside [{math.degrees(lower) + offset:.3f}, "
                                 f"{math.degrees(upper) + offset:.3f}] deg.")
        # Only eliminate floating point noise at a limit, never silently clamp
        # a user command outside the supported servo envelope.
        return np.clip(candidate, self._limits[:, 0], self._limits[:, 1])

    def _commit(self, candidate):
        validated = self._validate(candidate)
        self.data.ctrl[self.actuator_ids] = validated
        return self.target

    def set_joint_targets(self, yaw_deg=None, shoulder_deg=None, forearm_deg=None, grip_deg=None):
        """Set any subset of joint targets in degrees; reject bad input atomically.

        Shoulder=90 points vertically up. Forearm=0 points horizontally forward.
        Both arm angles are absolute elevations. Positive grip opens the jaws.
        """
        candidate = self.target
        for index, value in enumerate((yaw_deg, shoulder_deg, forearm_deg, grip_deg)):
            if value is not None:
                try:
                    candidate[index] = math.radians(float(value) - (90 if index == 1 else 0))
                except (TypeError, ValueError, OverflowError) as exc:
                    raise ValueError(f"{JOINT_LABELS[index]} angle must be finite degrees.") from exc
        return self._commit(candidate)

    def move_to(self, x_mm, y_mm, z_mm, grip_deg=None):
        """Set the closest valid IK branch for a TCP target in millimetres.

        The destination is validated, but this is not a collision-aware path
        planner. Servo dynamics move the mechanism toward the commanded pose.
        """
        current = self.target
        candidates = inverse_kinematics(x_mm, y_mm, z_mm, self.config)
        valid = []
        for angles in candidates:
            candidate = current.copy()
            candidate[:3] = angles
            if grip_deg is not None:
                try:
                    candidate[3] = math.radians(float(grip_deg))
                except (TypeError, ValueError, OverflowError) as exc:
                    raise ValueError("grip angle must be finite degrees.") from exc
            try:
                valid.append(self._validate(candidate))
            except ValueError:
                continue
        if not valid:
            raise ValueError("TCP position or grip target cannot be reached within the actuator limits.")
        closest = min(valid, key=lambda target: float(np.linalg.norm(target - current)))
        return self._commit(closest)

    def forward_kinematics(self, yaw_deg=None, shoulder_deg=None, forearm_deg=None):
        """Return ideal TCP in mm, defaulting omitted angles to current commands."""
        angles = self.target[:3]
        for index, value in enumerate((yaw_deg, shoulder_deg, forearm_deg)):
            if value is not None:
                try:
                    angles[index] = math.radians(float(value) - (90 if index == 1 else 0))
                except (TypeError, ValueError, OverflowError) as exc:
                    raise ValueError("Joint angles must be finite degrees.") from exc
        return forward_kinematics(angles, self.config)
