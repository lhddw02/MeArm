"""Measured, staged pick-and-place control for the physical cube scene.

All arm motion uses position actuators. Cube positions and velocities are
observations only: this module never writes free-joint state. Call ``update``
once before each MuJoCo physics step, instead of calling motion.update().
"""
from __future__ import annotations

import math
from pathlib import Path

if __package__:
    from ._runtime import configure_dependencies
    from .cube_scene import CUBES, CUBE_SIZE_M
else:
    from _runtime import configure_dependencies
    from cube_scene import CUBES, CUBE_SIZE_M

configure_dependencies(Path(__file__).resolve().parents[1])
import mujoco
import numpy as np


class PickPlaceTask:
    """Pick a named cube, place its center at a supported XYZ, then retreat.

    ``motion`` supplies model/data/arm, ``set_auto`` and ``update``. Optional
    callbacks receive this task; ``grasp_check`` returns a bool, while attach
    and release may manage a separately declared grasp backend. The default
    backend requires actual contacts with both finger collision geoms.
    """

    GRASP_OFFSET_MM = np.array([0.0, 0.0, 14.0])
    CLOSE_DEG = math.degrees(.3)
    POSITION_TOLERANCE_MM = 3.0
    SETTLE_SECONDS = 0.5

    def __init__(self, motion, grasp_check=None, attach=None, release=None):
        self.motion = motion
        self.model, self.data, self.arm = motion.model, motion.data, motion.arm
        self._grasp_check = grasp_check
        self._attach_callback, self._release_callback = attach, release
        self.active = False
        self.status, self.phase, self.error = 'idle', 'idle', ''
        self.color = None
        self.destination_mm = None
        self.history = []
        self.grasp_mode = 'contact'
        self._attached = False
        self._released = False
        self._plan = []
        self._phase_index = -1
        self._body_id = self._geom_id = self._joint_id = None
        self._settled_since = None
        self._grasp_offset_mm = None
        self._prelift_z_mm = None
        self._phase_started = 0.0
        self._phase_duration = 0.0
        self._phase_timeout = 0.0

    @property
    def cube_position_mm(self):
        if self._body_id is None:
            return None
        return 1000 * self.data.xpos[self._body_id].copy()

    @property
    def attached(self):
        return self._attached

    def _record(self, phase):
        self.history.append({'phase': phase, 'time': float(self.data.time)})

    def _velocity(self, joint_id):
        address = int(self.model.jnt_dofadr[joint_id])
        return self.data.qvel[address:address + 3], self.data.qvel[address + 3:address + 6]

    def _is_still(self, joint_id):
        linear, angular = self._velocity(joint_id)
        return np.linalg.norm(linear) <= 0.01 and np.linalg.norm(angular) <= 0.2

    def _table_bounds(self):
        gid = int(self.model.geom('platform_top').id)
        center = self.data.geom_xpos[gid]
        rotation = self.data.geom_xmat[gid].reshape(3, 3)
        extent = np.abs(rotation) @ self.model.geom_size[gid]
        return center - extent, center + extent

    def _free_after_release(self):
        if self._attached:
            return False
        if int(self.model.jnt_type[self._joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
            return False
        for index in range(self.model.neq):
            if not self.data.eq_active[index]:
                continue
            if int(self.model.eq_type[index]) in (int(mujoco.mjtEq.mjEQ_WELD), int(mujoco.mjtEq.mjEQ_CONNECT)):
                if self._body_id in (int(self.model.eq_obj1id[index]), int(self.model.eq_obj2id[index])):
                    return False
        return True

    def bilateral_contacts(self):
        """True only when the selected cube touches each physical finger pad."""
        if self._geom_id is None:
            return False
        fingers = set()
        for contact in self.data.contact:
            pair = (int(contact.geom1), int(contact.geom2))
            if self._geom_id not in pair:
                continue
            other = pair[1] if pair[0] == self._geom_id else pair[0]
            name = self.model.geom(other).name or ''
            if name.startswith('left_finger_') and ('collision' in name or 'pad' in name):
                fingers.add('left')
            if name.startswith('right_finger_') and ('collision' in name or 'pad' in name):
                fingers.add('right')
        return len(fingers) == 2

    def start(self, color, destination_mm, move_seconds=2.0, clearance_mm=35, grasp_mode='contact'):
        """Validate the entire job before changing commands or task state.

        Destinations describe the cube CENTER, on the existing tabletop.
        Unreachable targets, occupied destinations and unsettled source cubes
        raise ValueError and leave any current job and servo controls intact.
        """
        names = {cube['name'] for cube in CUBES}
        if not isinstance(color, str) or color.lower() not in names:
            raise ValueError('Cube color must be red, green, blue or yellow.')
        color = color.lower()
        try:
            destination = np.asarray(destination_mm, dtype=float)
            duration, clearance = float(move_seconds), float(clearance_mm)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Destination must contain three finite coordinates in mm.') from exc
        if destination.shape != (3,) or not np.all(np.isfinite(destination)):
            raise ValueError('Destination must contain three finite coordinates in mm.')
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('Move duration must be finite and greater than zero.')
        if not math.isfinite(clearance) or clearance < 20:
            raise ValueError('Clearance must be finite and at least 20 mm.')
        if grasp_mode != 'contact' and not (self._attach_callback and self._release_callback):
            raise ValueError('Only contact grasping is available in this scene.')
        body_id = int(self.model.body('cube_' + color).id)
        geom_id = int(self.model.geom('cube_' + color + '_geom').id)
        joint_id = int(self.model.joint('cube_' + color + '_joint').id)
        source = 1000 * self.data.xpos[body_id].copy()
        low, high = self._table_bounds()
        half = CUBE_SIZE_M / 2
        supported_z = 1000 * (high[2] + half)
        if abs(destination[2] - supported_z) > 0.5:
            raise ValueError(f'Destination cube center must be at tabletop height {supported_z:.1f} mm.')
        if np.any(destination[:2] / 1000 < low[:2] + half - 1e-10) or np.any(destination[:2] / 1000 > high[:2] - half + 1e-10):
            raise ValueError('Destination must keep the entire 20 mm cube on the tabletop.')
        if abs(source[2] - supported_z) > 1.0 or not self._is_still(joint_id):
            raise ValueError('Selected cube must be settled on the tabletop before picking it.')
        if np.any(source[:2] / 1000 < low[:2] + half - 1e-5) or np.any(source[:2] / 1000 > high[:2] - half + 1e-5):
            raise ValueError('Selected cube must be fully supported by the tabletop.')
        # Cube yaw changes with the arm while it is carried. Account for that
        # final orientation when computing the destination's world AABB.
        source_yaw = math.atan2(source[1], source[0])
        dest_yaw = math.atan2(destination[1], destination[0])
        turn = dest_yaw - source_yaw
        c, s = math.cos(turn), math.sin(turn)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        final_rotation = rotation @ self.data.geom_xmat[geom_id].reshape(3, 3)
        final_extent = np.abs(final_rotation) @ self.model.geom_size[geom_id]
        if np.any(destination[:2] / 1000 - final_extent[:2] < low[:2] - 1e-10) or np.any(destination[:2] / 1000 + final_extent[:2] > high[:2] + 1e-10):
            raise ValueError('The rotated cube would extend beyond the tabletop edge.')
        for other in names - {color}:
            other_id = int(self.model.geom('cube_' + other + '_geom').id)
            extent = np.abs(self.data.geom_xmat[other_id].reshape(3, 3)) @ self.model.geom_size[other_id]
            if np.all(np.abs(destination / 1000 - self.data.geom_xpos[other_id]) < final_extent + extent + .002):
                raise ValueError(f'Destination overlaps the {other} cube; leave at least 2 mm clearance.')
        grasp = source + np.asarray(self.GRASP_OFFSET_MM, dtype=float)
        place = destination + np.asarray(self.GRASP_OFFSET_MM, dtype=float)
        hover_source, hover_dest = grasp.copy(), place.copy()
        hover_source[2] += clearance
        hover_dest[2] += clearance
        open_deg = float(self.arm.limits_degrees[3, 1])
        current = self.arm.tcp_position_mm.copy()
        plan = [
            ('open', current, open_deg, max(.75, duration / 2)),
            ('approach', hover_source, open_deg, duration),
            ('descend', grasp, open_deg, duration),
            ('close', grasp, self.CLOSE_DEG, max(2.0, duration)),
            ('lift', hover_source, self.CLOSE_DEG, 1.5 * duration),
            ('transfer', hover_dest, self.CLOSE_DEG, 1.5 * duration),
            ('lower', place, self.CLOSE_DEG, 1.5 * duration),
            ('release', place, open_deg, max(1.0, duration)),
            ('retreat', hover_dest, open_deg, duration),
        ]
        # IK temporarily writes ctrl, so restore even when a later waypoint
        # fails. No trajectory/site/task state is touched until every IK passes.
        saved_ctrl = self.data.ctrl.copy()
        try:
            for phase, target, grip, _ in plan:
                try:
                    self.arm.move_to(*target, grip_deg=grip)
                except ValueError as exc:
                    raise ValueError(f'Cannot reach {phase} waypoint {np.round(target, 2).tolist()}: {exc}') from exc
        finally:
            self.data.ctrl[:] = saved_ctrl
        if self.active:
            raise ValueError('A pick-and-place job is already running; cancel it before starting another.')
        self.color = color
        self.destination_mm = destination.copy()
        self.grasp_mode = grasp_mode
        self._body_id, self._geom_id, self._joint_id = body_id, geom_id, joint_id
        self._plan = plan
        self._phase_index = -1
        self.error = ''
        self.history = []
        self._attached = False
        self._released = False
        self._settled_since = None
        self._grasp_offset_mm = None
        self._prelift_z_mm = None
        self.status, self.active = 'running', True
        try:
            marker = int(self.model.site('place_target').id)
        except (KeyError, ValueError):
            marker = None
        if marker is not None:
            self.model.site_pos[marker] = destination / 1000 - [0, 0, .0095]
            self.model.site_rgba[marker, 3] = .35
        self._next_phase()

    def _next_phase(self):
        self._phase_index += 1
        self._phase_started = float(self.data.time)
        if self._phase_index == len(self._plan):
            self.phase = 'settle'
            self._phase_duration = self.SETTLE_SECONDS
            self._phase_timeout = 4.0
            self._settled_since = None
        else:
            self.phase, target, grip, self._phase_duration = self._plan[self._phase_index]
            self.motion.set_auto(target, self._phase_duration, grip_deg=grip)
            self._phase_timeout = self._phase_duration + 4.0
        self._record(self.phase)

    def _release(self):
        if self._release_callback is not None:
            self._release_callback(self)
        self._attached = False

    def _fail(self, message):
        self._release()
        self.active, self.status, self.error = False, 'failed', str(message)
        self.phase = 'failed'
        self._record(self.phase)
        self.motion.set_manual()

    def cancel(self):
        """Stop the staged controller and return actuator ownership to manual."""
        self._release()
        self.motion.set_manual()
        if self.active:
            self.active, self.status, self.phase = False, 'cancelled', 'cancelled'
            self._record(self.phase)

    def reset(self):
        """Clear an inactive result after accepting a different control job.

        Servo commands, motion mode and all dynamic state are preserved. An
        active pick must be cancelled explicitly before its result is reset.
        """
        if self.active:
            raise ValueError('Cancel the active pick-and-place job before resetting its result.')
        self.__init__(self.motion, self._grasp_check,
                      self._attach_callback, self._release_callback)
        try:
            marker = int(self.model.site('place_target').id)
        except (KeyError, ValueError):
            return
        self.model.site_rgba[marker, 3] = 0

    def update(self):
        """Advance using simulation time and measured state, before mj_step."""
        self.motion.update()
        if not self.active:
            return
        elapsed = float(self.data.time) - self._phase_started
        cube = self.cube_position_mm
        if not np.all(np.isfinite(cube)):
            self._fail('Selected cube state became non-finite.')
            return
        if elapsed > self._phase_timeout:
            self._fail(f'Timed out during {self.phase}; the measured pose did not reach its target.')
            return
        if self.phase in ('lift', 'transfer', 'lower'):
            expected = self.arm.tcp_position_mm + self._grasp_offset_mm
            if np.linalg.norm(cube - expected) > 15:
                self._fail('Cube was lost from the gripper during ' + self.phase + '.')
                return
        if self.phase == 'settle':
            error = float(np.linalg.norm(cube - self.destination_mm))
            if self._released and self._free_after_release() and error <= self.POSITION_TOLERANCE_MM and self._is_still(self._joint_id):
                if self._settled_since is None:
                    self._settled_since = float(self.data.time)
                if self.data.time - self._settled_since >= self.SETTLE_SECONDS:
                    self.active, self.status, self.phase = False, 'complete', 'complete'
                    self._record(self.phase)
            else:
                self._settled_since = None
            return
        if not self.motion.finished:
            return
        # A cube between the pads can perturb the measured hand by a fraction
        # of a millimetre. A 3 mm threshold still catches collisions/misses.
        if self.motion.error_mm > self.POSITION_TOLERANCE_MM:
            return
        if self.phase == 'close':
            if elapsed < self._phase_duration + .35:
                return
            grasped = self._grasp_check(self) if self._grasp_check is not None else self.bilateral_contacts()
            if not grasped:
                if elapsed >= self._phase_duration + 1.0:
                    self._fail('Gripper missed the cube: both finger pads must touch before lifting.')
                return
            if self._attach_callback is not None:
                self._attach_callback(self)
                self._attached = True
            self._grasp_offset_mm = cube - self.arm.tcp_position_mm
            self._prelift_z_mm = float(cube[2])
        elif self.phase == 'lift':
            if cube[2] - self._prelift_z_mm < 15:
                self._fail('The cube did not lift at least 15 mm with the gripper.')
                return
        elif self.phase == 'release':
            if elapsed < self._phase_duration + .25:
                return
            self._release()
            if not self._free_after_release():
                self._fail('Cube is still constrained after the release command.')
                return
            if self.bilateral_contacts():
                return
            self._released = True
        self._next_phase()

    def report(self):
        """JSON-compatible task result for terminal status and batch runs."""
        cube = self.cube_position_mm
        return dict(status=self.status, phase=self.phase, active=self.active,
                    color=self.color, error=self.error, grasp_mode=self.grasp_mode,
                    destination_mm=None if self.destination_mm is None else self.destination_mm.tolist(),
                    cube_position_mm=None if cube is None else cube.tolist(),
                    placement_error_mm=None if cube is None or self.destination_mm is None else float(np.linalg.norm(cube - self.destination_mm)),
                    released=self._released and self._body_id is not None and self._free_after_release(),
                    history=list(self.history))
