"""Configurable multi-beam 3D lidar backed by MuJoCo ray casting."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from ..config import LidarConfig
from ..simulator import MujocoSimulator


class LidarSensor:
    def __init__(self, config: LidarConfig, simulator: MujocoSimulator, *, deterministic: bool, seed: int) -> None:
        self.config = config
        self.deterministic = deterministic
        self._rng = np.random.default_rng(seed)
        horizontal = np.linspace(-np.pi, np.pi, config.horizontal_samples, endpoint=False, dtype=np.float64)
        vertical = np.linspace(config.vertical_min_rad, config.vertical_max_rad, config.channels, endpoint=True, dtype=np.float64)
        vertical_grid, horizontal_grid = np.meshgrid(vertical, horizontal, indexing="ij")
        cosine_vertical = np.cos(vertical_grid)
        self.directions = np.column_stack(
            (
                (cosine_vertical * np.cos(horizontal_grid)).reshape(-1),
                (cosine_vertical * np.sin(horizontal_grid)).reshape(-1),
                np.sin(vertical_grid).reshape(-1),
            )
        )

    def sample(self, simulator: MujocoSimulator, timestamp_s: float) -> NDArray[np.float32]:
        distances = simulator.raycast_from_site("lidar_site", self.directions, self.config.max_range_m)
        valid = (distances >= self.config.min_range_m) & (distances <= self.config.max_range_m)
        selected_distances = distances[valid].copy()
        if not self.deterministic and self.config.range_noise_std_m > 0:
            selected_distances += self._rng.normal(0.0, self.config.range_noise_std_m, len(selected_distances))
            selected_distances = np.clip(selected_distances, self.config.min_range_m, self.config.max_range_m)
        xyz = self.directions[valid] * selected_distances[:, None]
        intensity = 255.0 * (1.0 - selected_distances / self.config.max_range_m)
        return np.ascontiguousarray(np.column_stack((xyz, intensity)), dtype=np.float32)
