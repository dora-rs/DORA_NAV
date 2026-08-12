#!/usr/bin/env python3
"""Continuously follow the static test world's wall loop using Odometry."""

from __future__ import annotations

import json
import sys
import time

import numpy as np
import pyarrow as pa
from dora import Node

try:
    from .waypoint_controller import OdometryError, Pose2D, VelocityCommand, WallLoopController, parse_imu_yaw, parse_odometry_motion
except ImportError:
    from waypoint_controller import OdometryError, Pose2D, VelocityCommand, WallLoopController, parse_imu_yaw, parse_odometry_motion


class CommandPublisher:
    def __init__(self, odometry_timeout_s: float = 0.5) -> None:
        self.odometry_timeout_s = odometry_timeout_s
        self.controller = WallLoopController()
        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad: float | None = None
        self._last_sample_timestamp_s: float | None = None
        self._last_odometry_s: float | None = None
        self._last_imu_s: float | None = None

    def on_odometry(self, payload: bytes, received_at_s: float) -> None:
        motion = parse_odometry_motion(payload)
        if self._last_sample_timestamp_s is not None:
            dt_s = motion.timestamp_s - self._last_sample_timestamp_s
            if dt_s < 0:
                raise OdometryError("Odometry timestamp moved backward")
            if self._yaw_rad is not None:
                self._x_m += motion.linear_x_mps * dt_s * np.cos(self._yaw_rad)
                self._y_m += motion.linear_x_mps * dt_s * np.sin(self._yaw_rad)
        self._last_sample_timestamp_s = motion.timestamp_s
        self._last_odometry_s = received_at_s

    def on_imu(self, payload: bytes, received_at_s: float) -> None:
        self._yaw_rad = parse_imu_yaw(payload)
        self._last_imu_s = received_at_s

    @property
    def estimated_pose(self) -> Pose2D:
        return Pose2D(self._x_m, self._y_m, self._yaw_rad or 0.0)

    @staticmethod
    def _serialize(command: VelocityCommand) -> bytes:
        return json.dumps(
            {"linear": {"x": command.linear_x_mps}, "angular": {"z": command.angular_z_rad_s}},
            separators=(",", ":"),
        ).encode("utf-8")

    def command_payload(self, now_s: float) -> bytes:
        if (
            self._last_sample_timestamp_s is None
            or self._yaw_rad is None
            or self._last_odometry_s is None
            or self._last_imu_s is None
            or now_s - self._last_odometry_s > self.odometry_timeout_s
            or now_s - self._last_imu_s > self.odometry_timeout_s
        ):
            return self._serialize(VelocityCommand(0.0, 0.0))
        return self._serialize(self.controller.update(self.estimated_pose))


def main() -> None:
    node = Node()
    publisher = CommandPublisher()
    for event in node:
        if event["type"] == "STOP":
            return
        if event["type"] != "INPUT":
            continue
        if event["id"] == "Odometry":
            try:
                publisher.on_odometry(bytes(event["value"].to_numpy(zero_copy_only=False)), time.monotonic())
            except (OdometryError, ValueError, TypeError) as exc:
                print(f"invalid Odometry ignored: {exc}", file=sys.stderr, flush=True)
        elif event["id"] == "imu":
            try:
                publisher.on_imu(bytes(event["value"].to_numpy(zero_copy_only=False)), time.monotonic())
            except (OdometryError, ValueError, TypeError) as exc:
                print(f"invalid IMU ignored: {exc}", file=sys.stderr, flush=True)
        elif event["id"] == "tick":
            payload = publisher.command_payload(time.monotonic())
            node.send_output(
                "cmd_vel",
                pa.array(np.frombuffer(payload, dtype=np.uint8), type=pa.uint8()),
            )


if __name__ == "__main__":
    main()
