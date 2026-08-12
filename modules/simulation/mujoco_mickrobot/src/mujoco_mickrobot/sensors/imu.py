"""IMU sampling from immutable MuJoCo snapshots."""

from __future__ import annotations

import math

import numpy as np

from ..config import ImuConfig
from ..messages import ImuSample
from ..simulator import SimulationSnapshot


STANDARD_GRAVITY_MPS2 = 9.80665


class ImuSensor:
    def __init__(self, config: ImuConfig, *, deterministic: bool, seed: int) -> None:
        self.config = config
        self.deterministic = deterministic
        self._rng = np.random.default_rng(seed)
        self._accel_bias = np.asarray(config.acceleration_bias_g, dtype=np.float64)
        self._gyro_bias = np.asarray(config.gyro_bias_rad_s, dtype=np.float64)
        self._last_timestamp_s: float | None = None

    def sample(self, snapshot: SimulationSnapshot, timestamp_s: float) -> ImuSample:
        acceleration = np.asarray(snapshot.imu_linear_acceleration_mps2, dtype=np.float64) / STANDARD_GRAVITY_MPS2
        angular_velocity = np.asarray(snapshot.imu_angular_velocity_rad_s, dtype=np.float64)
        if not self.deterministic:
            if self._last_timestamp_s is not None:
                dt = max(0.0, timestamp_s - self._last_timestamp_s)
                walk = self.config.bias_random_walk_std * math.sqrt(dt)
                self._accel_bias += self._rng.normal(0.0, walk, 3)
                self._gyro_bias += self._rng.normal(0.0, walk, 3)
            acceleration += self._accel_bias + self._rng.normal(0.0, self.config.acceleration_noise_std_g, 3)
            angular_velocity += self._gyro_bias + self._rng.normal(0.0, self.config.gyro_noise_std_rad_s, 3)
        self._last_timestamp_s = timestamp_s
        w, x, y, z = snapshot.imu_orientation_wxyz
        return ImuSample(
            timestamp_s,
            self.config.frame_id,
            (x, y, z, w),
            tuple(float(value) for value in angular_velocity),  # type: ignore[arg-type]
            tuple(float(value) for value in acceleration),  # type: ignore[arg-type]
        )
