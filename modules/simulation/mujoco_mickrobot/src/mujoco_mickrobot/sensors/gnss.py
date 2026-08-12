"""GNSS sampling from MuJoCo ENU ground truth."""

from __future__ import annotations

import numpy as np

from ..config import GnssConfig
from ..geo import Wgs84Origin, enu_to_wgs84
from ..messages import GnssSample
from ..simulator import SimulationSnapshot


class GnssSensor:
    def __init__(self, config: GnssConfig, *, deterministic: bool, seed: int) -> None:
        self.config = config
        self.deterministic = deterministic
        self._rng = np.random.default_rng(seed)
        self._origin = Wgs84Origin(config.origin.latitude_deg, config.origin.longitude_deg, config.origin.altitude_m)

    def sample(self, snapshot: SimulationSnapshot, timestamp_s: float) -> GnssSample:
        position = np.asarray(snapshot.gnss_position_m, dtype=np.float64)
        velocity = np.asarray(snapshot.gnss_velocity_mps, dtype=np.float64)
        if not self.deterministic:
            position += self._rng.normal(0.0, self.config.position_noise_std_m, 3)
            velocity += self._rng.normal(0.0, self.config.velocity_noise_std_mps, 3)
        latitude, longitude, altitude = enu_to_wgs84(position, self._origin)
        variance = self.config.position_noise_std_m**2
        covariance = (variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance)
        return GnssSample(
            timestamp_s,
            self.config.frame_id,
            True,
            latitude,
            longitude,
            altitude,
            tuple(float(value) for value in velocity),  # type: ignore[arg-type]
            covariance,
        )
