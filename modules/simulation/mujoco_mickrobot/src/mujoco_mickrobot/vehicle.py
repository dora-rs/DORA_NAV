"""Safe command parsing and differential-drive wheel control."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any

from .config import VehicleConfig


class CommandError(ValueError):
    """Raised when a cmd_vel payload violates the input contract."""


@dataclass(frozen=True)
class TwistCommand:
    linear_x_mps: float
    angular_z_rad_s: float


@dataclass(frozen=True)
class WheelTargets:
    left_front_rad_s: float
    left_back_rad_s: float
    right_front_rad_s: float
    right_back_rad_s: float

    @classmethod
    def zero(cls) -> "WheelTargets":
        return cls(0.0, 0.0, 0.0, 0.0)

    @classmethod
    def uniform(cls, speed_rad_s: float) -> "WheelTargets":
        value = float(speed_rad_s)
        return cls(value, value, value, value)

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.left_front_rad_s,
            self.left_back_rad_s,
            self.right_front_rad_s,
            self.right_back_rad_s,
        )


def _reject_constant(value: str) -> Any:
    raise CommandError(f"cmd_vel contains invalid number: {value}")


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise CommandError(f"{path} must be finite")
    return result


def parse_cmd_vel(payload: bytes) -> TwistCommand:
    """Parse the repository cmd_vel JSON contract without numeric coercion."""
    try:
        value = json.loads(payload.decode("utf-8"), parse_constant=_reject_constant)
    except CommandError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CommandError(f"cmd_vel is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise CommandError("cmd_vel root must be an object")
    linear = value.get("linear")
    angular = value.get("angular")
    if not isinstance(linear, dict) or not isinstance(angular, dict):
        raise CommandError("cmd_vel requires linear and angular objects")
    if "x" not in linear or "z" not in angular:
        raise CommandError("cmd_vel requires linear.x and angular.z")
    return TwistCommand(
        linear_x_mps=_number(linear["x"], "linear.x"),
        angular_z_rad_s=_number(angular["z"], "angular.z"),
    )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _slew(current: float, target: float, maximum_delta: float) -> float:
    return current + _clamp(target - current, maximum_delta)


class VehicleController:
    """Convert chassis commands to four wheel targets with a safety timeout."""

    def __init__(self, config: VehicleConfig) -> None:
        self.config = config
        self._command: TwistCommand | None = None
        self._received_at_s: float | None = None
        self._current = WheelTargets.zero()
        self.timed_out = False

    def apply(self, command: TwistCommand, received_at_s: float) -> None:
        if not math.isfinite(received_at_s):
            raise CommandError("received_at_s must be finite")
        if not math.isfinite(command.linear_x_mps) or not math.isfinite(command.angular_z_rad_s):
            raise CommandError("command values must be finite")
        self._command = command
        self._received_at_s = received_at_s
        self.timed_out = False

    def _desired(self) -> WheelTargets:
        if self._command is None:
            return WheelTargets.zero()
        linear = _clamp(self._command.linear_x_mps, self.config.max_linear_mps)
        angular = _clamp(self._command.angular_z_rad_s, self.config.max_angular_rad_s)
        half_track = self.config.wheel_separation_m / 2.0
        left = (linear - angular * half_track) / self.config.wheel_radius_m
        right = (linear + angular * half_track) / self.config.wheel_radius_m
        left = _clamp(left, self.config.max_wheel_rad_s)
        right = _clamp(right, self.config.max_wheel_rad_s)
        return WheelTargets(left, left, right, right)

    def update(self, now_s: float, dt_s: float) -> WheelTargets:
        if not math.isfinite(now_s) or not math.isfinite(dt_s) or dt_s < 0:
            raise CommandError("now_s must be finite and dt_s must be finite and non-negative")
        if self._received_at_s is None or now_s - self._received_at_s > self.config.cmd_timeout_s:
            self.timed_out = self._received_at_s is not None
            self._current = WheelTargets.zero()
            return self._current
        desired = self._desired()
        maximum_delta = self.config.max_wheel_accel_rad_s2 * dt_s
        self._current = WheelTargets(*(
            _slew(current, target, maximum_delta)
            for current, target in zip(self._current.as_tuple(), desired.as_tuple())
        ))
        return self._current

    def stop(self) -> WheelTargets:
        self._command = None
        self._received_at_s = None
        self._current = WheelTargets.zero()
        self.timed_out = False
        return self._current
