"""Wheel-feedback odometry independent from MuJoCo ground truth."""

from __future__ import annotations

import math

import numpy as np

from ..config import OdometryConfig, VehicleConfig
from ..messages import OdometrySample
from ..simulator import SimulationSnapshot


class WheelOdometry:
    def __init__(self, config: OdometryConfig, vehicle: VehicleConfig, *, deterministic: bool, seed: int) -> None:
        self.config = config
        self.vehicle = vehicle
        self.deterministic = deterministic
        self._rng = np.random.default_rng(seed)
        self.reset()

    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_timestamp_s: float | None = None
        if self.deterministic:
            self._left_scale = self._right_scale = 1.0
        else:
            self._left_scale, self._right_scale = 1.0 + self._rng.normal(0.0, self.config.wheel_scale_error_std, 2)

    def sample(self, snapshot: SimulationSnapshot, timestamp_s: float) -> OdometrySample:
        wheels = np.asarray(snapshot.wheel_velocity_rad_s, dtype=np.float64)
        if not self.deterministic:
            wheels += self._rng.normal(0.0, self.config.wheel_noise_std_rad_s, 4)
        left = float((wheels[0] + wheels[1]) * 0.5 * self._left_scale)
        right = float((wheels[2] + wheels[3]) * 0.5 * self._right_scale)
        linear = self.vehicle.wheel_radius_m * (left + right) * 0.5
        angular = self.vehicle.wheel_radius_m * (right - left) / self.vehicle.wheel_separation_m
        if self._last_timestamp_s is not None:
            dt = timestamp_s - self._last_timestamp_s
            if dt < 0:
                raise ValueError("odometry timestamp moved backward")
            delta_yaw = angular * dt
            heading = self._yaw + delta_yaw * 0.5
            self._x += linear * math.cos(heading) * dt
            self._y += linear * math.sin(heading) * dt
            self._yaw += delta_yaw
        self._last_timestamp_s = timestamp_s
        half_yaw = self._yaw * 0.5
        return OdometrySample(
            timestamp_s,
            self.config.frame_id,
            self.config.child_frame_id,
            (self._x, self._y, 0.0),
            (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
            (linear, 0.0, 0.0),
            (0.0, 0.0, angular),
        )
