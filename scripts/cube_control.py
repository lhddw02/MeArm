"""Shared manual/automatic joint trajectories for both cube-scene viewers."""
from __future__ import annotations

import math

if __package__:
    from .arm_control import MeArmController
else:
    from arm_control import MeArmController
import numpy as np


class CubeSceneController:
    """Two control modes sharing the same four position actuators."""

    def __init__(self, model, data):
        self.model, self.data = model, data
        self.arm = MeArmController(model, data)
        self.mode = 'manual'
        self.target_mm = None
        self._start = self._goal = None
        self._start_time = 0.0
        self._move_seconds = 2.0
        self._target_site = int(model.site('reach_target').id)
        self.model.site_rgba[self._target_site, 3] = 0

    def set_manual(self):
        self.mode = 'manual'
        self.model.site_rgba[self._target_site, 3] = 0

    def set_auto(self, target_mm, move_seconds=2.0, grip_deg=None):
        duration = float(move_seconds)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('Move duration must be finite and greater than zero.')
        target = np.asarray(target_mm, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('Target must be three finite world coordinates in millimeters.')
        start = self.arm.target
        try:
            goal = self.arm.move_to(*target, grip_deg=grip_deg)
        finally:
            self.data.ctrl[self.arm.actuator_ids] = start
        self._start, self._goal = start, goal
        self._start_time = float(self.data.time)
        self._move_seconds = duration
        self.target_mm = target.copy()
        self.model.site_pos[self._target_site] = target / 1000
        self.model.site_rgba[self._target_site, 3] = .65
        self.mode = 'auto'

    @property
    def finished(self):
        return self.mode == 'auto' and self.data.time - self._start_time >= self._move_seconds - 1e-12

    @property
    def error_mm(self):
        if self.mode != 'auto' or self.target_mm is None:
            return math.inf
        return float(np.linalg.norm(self.arm.tcp_position_mm - self.target_mm))

    def update(self):
        if self.mode != 'auto':
            return
        t = float(np.clip((self.data.time - self._start_time) / self._move_seconds, 0, 1))
        blend = t*t*t*(10+t*(-15+6*t))
        self.data.ctrl[self.arm.actuator_ids] = self._start + blend * (self._goal - self._start)
