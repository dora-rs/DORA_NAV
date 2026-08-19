"""MuJoCo lifecycle and immutable simulation snapshots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import mujoco
import numpy as np

from .config import SimulationConfig
from .model_loader import MODEL_PATH, load_generated_model
from .vehicle import WheelTargets


WHEEL_JOINT_ORDER = (
    "left_front_wheel_joint",
    "left_back_wheel_joint",
    "right_front_wheel_joint",
    "right_back_wheel_joint",
)
WHEEL_ACTUATOR_NAMES = tuple(name.removesuffix("_joint") + "_actuator" for name in WHEEL_JOINT_ORDER)


class SimulatorError(RuntimeError):
    """Raised when MuJoCo resources or lifecycle operations fail."""


@dataclass(frozen=True)
class SimulationSnapshot:
    timestamp_s: float
    position_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    linear_velocity_world_mps: tuple[float, float, float]
    angular_velocity_world_rad_s: tuple[float, float, float]
    wheel_velocity_rad_s: tuple[float, float, float, float]
    imu_orientation_wxyz: tuple[float, float, float, float]
    imu_angular_velocity_rad_s: tuple[float, float, float]
    imu_linear_acceleration_mps2: tuple[float, float, float]
    gnss_position_m: tuple[float, float, float]
    gnss_velocity_mps: tuple[float, float, float]

    @classmethod
    def stationary(cls) -> "SimulationSnapshot":
        return cls(
            0.0,
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    def numeric_values(self) -> Iterable[float]:
        yield self.timestamp_s
        for values in (
            self.position_m,
            self.orientation_wxyz,
            self.linear_velocity_world_mps,
            self.angular_velocity_world_rad_s,
            self.wheel_velocity_rad_s,
            self.imu_orientation_wxyz,
            self.imu_angular_velocity_rad_s,
            self.imu_linear_acceleration_mps2,
            self.gnss_position_m,
            self.gnss_velocity_mps,
        ):
            yield from values


def _tuple(values, length: int):
    copied = tuple(float(value) for value in np.asarray(values).reshape(-1))
    if len(copied) != length:
        raise SimulatorError(f"expected {length} values, got {len(copied)}")
    return copied


class MujocoSimulator:
    """Own an MjModel/MjData pair and expose copied state only."""

    def __init__(self, config: SimulationConfig, model_path=MODEL_PATH) -> None:
        self.config = config
        self.model = load_generated_model(model_path)
        self.model.opt.timestep = config.physics.timestep_s
        self.model.opt.gravity[:] = config.physics.gravity_mps2
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        if camera_id >= 0:
            horizontal_fov_rad = math.radians(config.camera.horizontal_fov_deg)
            self.model.cam_fovy[camera_id] = math.degrees(
                2.0 * math.atan(math.tan(horizontal_fov_rad / 2.0) * config.camera.height / config.camera.width)
            )
        self.data = mujoco.MjData(self.model)
        self.wheel_actuators = WHEEL_ACTUATOR_NAMES
        self._actuator_ids = tuple(self._name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in WHEEL_ACTUATOR_NAMES)
        force_limit = config.vehicle.actuator_force_limit_n
        for actuator_id in self._actuator_ids:
            self.model.actuator_forcelimited[actuator_id] = 1
            self.model.actuator_forcerange[actuator_id] = (-force_limit, force_limit)
        self._wheel_dof_addresses = tuple(int(self.model.joint(name).dofadr[0]) for name in WHEEL_JOINT_ORDER)
        self._base_joint = self.model.joint("base_freejoint")
        self._sensor_slices: dict[str, slice] = {}
        if config.lidar.enabled:
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, "lidar_site")
        if config.camera.enabled:
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, "camera_site")
            self._name_id(mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        if config.imu.enabled:
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, "imu_site")
            for name in ("imu_orientation", "imu_gyro", "imu_accelerometer"):
                self._sensor_slices[name] = self._sensor_slice(name)
        if config.gnss.enabled:
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, "gnss_site")
            for name in ("gnss_position", "gnss_velocity"):
                self._sensor_slices[name] = self._sensor_slice(name)
        self._closed = False
        self._viewer = None
        mujoco.mj_forward(self.model, self.data)

    def _name_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        identifier = mujoco.mj_name2id(self.model, object_type, name)
        if identifier < 0:
            raise SimulatorError(f"required MuJoCo resource is missing: {name}")
        return identifier

    def _sensor_slice(self, name: str) -> slice:
        sensor = self.model.sensor(name)
        start = int(sensor.adr[0])
        return slice(start, start + int(sensor.dim[0]))

    @property
    def time_s(self) -> float:
        return float(self.data.time)

    @property
    def controls(self) -> tuple[float, float, float, float]:
        return tuple(float(self.data.ctrl[index]) for index in self._actuator_ids)  # type: ignore[return-value]

    def step(self, targets: WheelTargets) -> None:
        if self._closed:
            raise SimulatorError("simulator is closed")
        for actuator_id, target in zip(self._actuator_ids, targets.as_tuple()):
            if not math.isfinite(target):
                raise SimulatorError("wheel targets must be finite")
            self.data.ctrl[actuator_id] = target
        mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:
        if self._closed:
            raise SimulatorError("simulator is closed")
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def snapshot(self) -> SimulationSnapshot:
        if self._closed:
            raise SimulatorError("simulator is closed")
        qpos = int(self._base_joint.qposadr[0])
        dof = int(self._base_joint.dofadr[0])
        sensor = self.data.sensordata
        imu_orientation = _tuple(sensor[self._sensor_slices["imu_orientation"]], 4) if "imu_orientation" in self._sensor_slices else (1.0, 0.0, 0.0, 0.0)
        imu_gyro = _tuple(sensor[self._sensor_slices["imu_gyro"]], 3) if "imu_gyro" in self._sensor_slices else (0.0, 0.0, 0.0)
        imu_acceleration = _tuple(sensor[self._sensor_slices["imu_accelerometer"]], 3) if "imu_accelerometer" in self._sensor_slices else (0.0, 0.0, 0.0)
        gnss_position = _tuple(sensor[self._sensor_slices["gnss_position"]], 3) if "gnss_position" in self._sensor_slices else (0.0, 0.0, 0.0)
        gnss_velocity = _tuple(sensor[self._sensor_slices["gnss_velocity"]], 3) if "gnss_velocity" in self._sensor_slices else (0.0, 0.0, 0.0)
        return SimulationSnapshot(
            timestamp_s=self.time_s,
            position_m=_tuple(self.data.qpos[qpos : qpos + 3], 3),
            orientation_wxyz=_tuple(self.data.qpos[qpos + 3 : qpos + 7], 4),
            linear_velocity_world_mps=_tuple(self.data.qvel[dof : dof + 3], 3),
            angular_velocity_world_rad_s=_tuple(self.data.qvel[dof + 3 : dof + 6], 3),
            wheel_velocity_rad_s=tuple(float(self.data.qvel[address]) for address in self._wheel_dof_addresses),  # type: ignore[arg-type]
            imu_orientation_wxyz=imu_orientation,
            imu_angular_velocity_rad_s=imu_gyro,
            imu_linear_acceleration_mps2=imu_acceleration,
            gnss_position_m=gnss_position,
            gnss_velocity_mps=gnss_velocity,
        )

    def raycast_from_site(
        self,
        site_name: str,
        directions_local: np.ndarray,
        cutoff_m: float,
    ) -> np.ndarray:
        """Cast world-only rays and return one distance per local direction."""
        if self._closed:
            raise SimulatorError("simulator is closed")
        site_id = self._name_id(mujoco.mjtObj.mjOBJ_SITE, site_name)
        directions = np.asarray(directions_local, dtype=np.float64)
        if directions.ndim != 2 or directions.shape[1] != 3:
            raise SimulatorError(f"ray directions must have shape (N, 3), got {directions.shape}")
        rotation = np.asarray(self.data.site_xmat[site_id], dtype=np.float64).reshape(3, 3)
        world_directions = np.ascontiguousarray(directions @ rotation.T)
        distances = np.empty(len(directions), dtype=np.float64)
        geom_ids = np.empty(len(directions), dtype=np.int32)
        world_geom_group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            self.data.site_xpos[site_id],
            world_directions.reshape(-1),
            world_geom_group,
            True,
            -1,
            geom_ids,
            distances,
            None,
            len(directions),
            cutoff_m,
        )
        return distances

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        self._closed = True

    def launch_viewer(self) -> None:
        if self._closed:
            raise SimulatorError("simulator is closed")
        if self._viewer is not None:
            return
        try:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        except Exception as exc:
            raise SimulatorError(f"failed to initialize MuJoCo GUI: {exc}") from exc

    def viewer_is_running(self) -> bool:
        return self._viewer is None or bool(self._viewer.is_running())

    def sync_viewer(self) -> None:
        if self._viewer is not None:
            self._viewer.sync()
