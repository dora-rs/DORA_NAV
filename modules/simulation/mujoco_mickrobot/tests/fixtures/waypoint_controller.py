"""Odometry-feedback controller for the static MuJoCo wall loop."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any


STARTUP_WAYPOINT = (0.0, -8.5)
LOOP_WAYPOINTS = (
    (-8.5, -8.5),
    (-8.5, 8.5),
    (8.5, 8.5),
    (8.5, -8.5),
    (0.0, -8.5),
)


class OdometryError(ValueError):
    """Raised when an Odometry payload is not a finite planar pose."""


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class VelocityCommand:
    linear_x_mps: float
    angular_z_rad_s: float


@dataclass(frozen=True)
class OdometryMotion:
    timestamp_s: float
    linear_x_mps: float


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise OdometryError(f"{name} must be an object")
    return value


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OdometryError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise OdometryError(f"{name} must be finite")
    return number


def _json_root(payload: bytes, name: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OdometryError(f"{name} is not valid UTF-8 JSON: {exc}") from exc
    return _object(value, name)


def parse_odometry_motion(payload: bytes) -> OdometryMotion:
    root = _json_root(payload, "Odometry")
    header = _object(root.get("header"), "header")
    twist = _object(root.get("twist"), "twist")
    linear = _object(twist.get("linear"), "twist.linear")
    return OdometryMotion(
        _number(header.get("timestamp"), "header.timestamp"),
        _number(linear.get("x"), "twist.linear.x"),
    )


def parse_imu_yaw(payload: bytes) -> float:
    root = _json_root(payload, "IMU")
    orientation = _object(root.get("orientation"), "orientation")
    z = _number(orientation.get("z"), "pose.orientation.z")
    w = _number(orientation.get("w"), "pose.orientation.w")
    norm = math.hypot(z, w)
    if norm < 1e-12:
        raise OdometryError("planar orientation quaternion cannot be zero")
    yaw = 2.0 * math.atan2(z / norm, w / norm)
    return math.atan2(math.sin(yaw), math.cos(yaw))


class WallLoopController:
    waypoint_tolerance_m = 0.35
    maximum_linear_mps = 0.6
    maximum_angular_rad_s = 0.8
    angular_gain = 1.5
    stop_heading_error_rad = math.radians(20.0)
    slowdown_distance_m = 1.0

    def __init__(self) -> None:
        self.startup_complete = False
        self.loop_index = 0

    @property
    def target(self) -> tuple[float, float]:
        return LOOP_WAYPOINTS[self.loop_index] if self.startup_complete else STARTUP_WAYPOINT

    def _advance_if_reached(self, pose: Pose2D) -> None:
        for _ in range(len(LOOP_WAYPOINTS) + 1):
            target_x, target_y = self.target
            if math.hypot(target_x - pose.x_m, target_y - pose.y_m) >= self.waypoint_tolerance_m:
                return
            if not self.startup_complete:
                self.startup_complete = True
                self.loop_index = 0
            else:
                self.loop_index = (self.loop_index + 1) % len(LOOP_WAYPOINTS)

    def update(self, pose: Pose2D) -> VelocityCommand:
        self._advance_if_reached(pose)
        target_x, target_y = self.target
        delta_x = target_x - pose.x_m
        delta_y = target_y - pose.y_m
        distance = math.hypot(delta_x, delta_y)
        target_yaw = math.atan2(delta_y, delta_x)
        heading_error = math.atan2(math.sin(target_yaw - pose.yaw_rad), math.cos(target_yaw - pose.yaw_rad))
        angular = max(-self.maximum_angular_rad_s, min(self.maximum_angular_rad_s, self.angular_gain * heading_error))
        if abs(heading_error) > self.stop_heading_error_rad:
            linear = 0.0
        else:
            distance_scale = min(1.0, distance / self.slowdown_distance_m)
            heading_scale = max(0.0, math.cos(heading_error))
            linear = self.maximum_linear_mps * distance_scale * heading_scale
        return VelocityCommand(linear, angular)
