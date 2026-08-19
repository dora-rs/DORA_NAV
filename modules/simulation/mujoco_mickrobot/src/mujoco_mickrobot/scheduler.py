"""Drift-free simulation-time sensor scheduling."""

from __future__ import annotations

import math


class ScheduleError(ValueError):
    """Raised for an invalid schedule or non-monotonic clock."""


class PeriodicSchedule:
    """Return exact period timestamps crossed by a monotonic simulation clock."""

    def __init__(self, rate_hz: float, start_s: float = 0.0) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ScheduleError("rate_hz must be finite and positive")
        if not math.isfinite(start_s):
            raise ScheduleError("start_s must be finite")
        self.rate_hz = float(rate_hz)
        self.start_s = float(start_s)
        self._next_index = 0
        self._last_now_s = -math.inf

    def due(self, now_s: float) -> tuple[float, ...]:
        if not math.isfinite(now_s):
            raise ScheduleError("now_s must be finite")
        if now_s < self._last_now_s - 1e-12:
            raise ScheduleError(f"simulation time moved backward: {now_s} < {self._last_now_s}")
        self._last_now_s = now_s
        if now_s < self.start_s - 1e-12:
            return ()
        last_due = math.floor((now_s - self.start_s + 1e-12) * self.rate_hz)
        if last_due < self._next_index:
            return ()
        timestamps = tuple(
            self.start_s + index / self.rate_hz
            for index in range(self._next_index, last_due + 1)
        )
        self._next_index = last_due + 1
        return timestamps

    def reset(self, start_s: float = 0.0) -> None:
        if not math.isfinite(start_s):
            raise ScheduleError("start_s must be finite")
        self.start_s = float(start_s)
        self._next_index = 0
        self._last_now_s = -math.inf
