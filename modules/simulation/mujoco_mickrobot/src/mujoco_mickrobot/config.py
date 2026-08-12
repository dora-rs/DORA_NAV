"""Strict configuration loading for the MickRobot simulator."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


MODULE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = MODULE_ROOT / "config" / "default.yaml"


class ConfigError(ValueError):
    """Raised when configuration is malformed or inconsistent."""


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class PhysicsConfig:
    timestep_s: float
    gravity_mps2: Vector3
    realtime_factor: float


@dataclass(frozen=True)
class VehicleConfig:
    wheel_separation_m: float
    wheel_radius_m: float
    cmd_timeout_s: float
    max_linear_mps: float
    max_angular_rad_s: float
    max_wheel_rad_s: float
    max_wheel_accel_rad_s2: float
    actuator_force_limit_n: float


@dataclass(frozen=True)
class LidarConfig:
    enabled: bool
    rate_hz: float
    frame_id: str
    channels: int
    horizontal_samples: int
    vertical_min_rad: float
    vertical_max_rad: float
    min_range_m: float
    max_range_m: float
    range_noise_std_m: float


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool
    rate_hz: float
    frame_id: str
    width: int
    height: int
    horizontal_fov_deg: float

    @property
    def resolution(self) -> tuple[int, int]:
        return self.width, self.height


@dataclass(frozen=True)
class ImuConfig:
    enabled: bool
    rate_hz: float
    frame_id: str
    acceleration_noise_std_g: float
    gyro_noise_std_rad_s: float
    acceleration_bias_g: Vector3
    gyro_bias_rad_s: Vector3
    bias_random_walk_std: float


@dataclass(frozen=True)
class OdometryConfig:
    enabled: bool
    rate_hz: float
    frame_id: str
    child_frame_id: str
    wheel_scale_error_std: float
    wheel_noise_std_rad_s: float


@dataclass(frozen=True)
class GnssOriginConfig:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class GnssConfig:
    enabled: bool
    rate_hz: float
    frame_id: str
    origin: GnssOriginConfig
    position_noise_std_m: float
    velocity_noise_std_mps: float


@dataclass(frozen=True)
class RuntimeConfig:
    headless: bool
    fastest: bool
    deterministic: bool
    random_seed: int
    output_failure_limit: int
    statistics_interval_s: float


@dataclass(frozen=True)
class RerunConfig:
    spawn: bool
    connect_url: str
    rate_window_s: float
    statistics_interval_s: float


@dataclass(frozen=True)
class SimulationConfig:
    physics: PhysicsConfig
    vehicle: VehicleConfig
    lidar: LidarConfig
    camera: CameraConfig
    imu: ImuConfig
    odometry: OdometryConfig
    gnss: GnssConfig
    runtime: RuntimeConfig
    rerun: RerunConfig


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load configuration {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise ConfigError(f"configuration root must be a mapping: {path}")
    return loaded


def _deep_merge(default: dict[str, Any], override: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    result = copy.deepcopy(default)
    for key, value in override.items():
        dotted = f"{prefix}.{key}" if prefix else str(key)
        if key not in default:
            raise ConfigError(f"unknown configuration key: {dotted}")
        if isinstance(default[key], dict):
            if not isinstance(value, dict):
                raise ConfigError(f"configuration section must be a mapping: {dotted}")
            result[key] = _deep_merge(default[key], value, dotted)
        else:
            result[key] = value
    return result


def _vector3(value: Any, name: str) -> Vector3:
    if not isinstance(value, list) or len(value) != 3 or any(isinstance(v, bool) for v in value):
        raise ConfigError(f"{name} must contain three numbers")
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} must contain three numbers") from exc
    if not all(math.isfinite(number) for number in result):
        raise ConfigError(f"{name} must contain three finite numbers")
    return result  # type: ignore[return-value]


def _positive(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{name} must be positive")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} must be positive") from exc
    if number <= 0:
        raise ConfigError(f"{name} must be positive")
    return number


def _nonnegative(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{name} must be non-negative")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} must be non-negative") from exc
    if number < 0:
        raise ConfigError(f"{name} must be non-negative")
    return number


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ConfigError(f"{name} must be a positive integer")
    return value


def _bool_env(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ConfigError(f"{name} must be one of 0/1, false/true, no/yes, off/on")


def _build_config(raw: dict[str, Any]) -> SimulationConfig:
    physics = raw["physics"]
    vehicle = raw["vehicle"]
    lidar = raw["lidar"]
    camera = raw["camera"]
    imu = raw["imu"]
    odometry = raw["odometry"]
    gnss = raw["gnss"]
    runtime = raw["runtime"]
    rerun = raw["rerun"]

    min_range = _nonnegative(lidar["min_range_m"], "lidar.min_range_m")
    max_range = _positive(lidar["max_range_m"], "lidar.max_range_m")
    if min_range >= max_range:
        raise ConfigError("lidar.min_range_m must be less than lidar.max_range_m")
    vertical_min = float(lidar["vertical_min_rad"])
    vertical_max = float(lidar["vertical_max_rad"])
    if vertical_min >= vertical_max:
        raise ConfigError("lidar.vertical_min_rad must be less than lidar.vertical_max_rad")

    width = _positive_int(camera["width"], "camera.width")
    height = _positive_int(camera["height"], "camera.height")
    horizontal_fov = _positive(camera["horizontal_fov_deg"], "camera.horizontal_fov_deg")
    if horizontal_fov >= 180:
        raise ConfigError("camera.horizontal_fov_deg must be less than 180")

    return SimulationConfig(
        physics=PhysicsConfig(
            timestep_s=_positive(physics["timestep_s"], "physics.timestep_s"),
            gravity_mps2=_vector3(physics["gravity_mps2"], "physics.gravity_mps2"),
            realtime_factor=_positive(physics["realtime_factor"], "physics.realtime_factor"),
        ),
        vehicle=VehicleConfig(
            wheel_separation_m=_positive(vehicle["wheel_separation_m"], "vehicle.wheel_separation_m"),
            wheel_radius_m=_positive(vehicle["wheel_radius_m"], "vehicle.wheel_radius_m"),
            cmd_timeout_s=_positive(vehicle["cmd_timeout_s"], "vehicle.cmd_timeout_s"),
            max_linear_mps=_positive(vehicle["max_linear_mps"], "vehicle.max_linear_mps"),
            max_angular_rad_s=_positive(vehicle["max_angular_rad_s"], "vehicle.max_angular_rad_s"),
            max_wheel_rad_s=_positive(vehicle["max_wheel_rad_s"], "vehicle.max_wheel_rad_s"),
            max_wheel_accel_rad_s2=_positive(vehicle["max_wheel_accel_rad_s2"], "vehicle.max_wheel_accel_rad_s2"),
            actuator_force_limit_n=_positive(vehicle["actuator_force_limit_n"], "vehicle.actuator_force_limit_n"),
        ),
        lidar=LidarConfig(
            enabled=bool(lidar["enabled"]), rate_hz=_positive(lidar["rate_hz"], "lidar.rate_hz"),
            frame_id=str(lidar["frame_id"]), channels=_positive_int(lidar["channels"], "lidar.channels"),
            horizontal_samples=_positive_int(lidar["horizontal_samples"], "lidar.horizontal_samples"),
            vertical_min_rad=vertical_min, vertical_max_rad=vertical_max,
            min_range_m=min_range, max_range_m=max_range,
            range_noise_std_m=_nonnegative(lidar["range_noise_std_m"], "lidar.range_noise_std_m"),
        ),
        camera=CameraConfig(
            enabled=bool(camera["enabled"]), rate_hz=_positive(camera["rate_hz"], "camera.rate_hz"),
            frame_id=str(camera["frame_id"]), width=width, height=height,
            horizontal_fov_deg=horizontal_fov,
        ),
        imu=ImuConfig(
            enabled=bool(imu["enabled"]), rate_hz=_positive(imu["rate_hz"], "imu.rate_hz"),
            frame_id=str(imu["frame_id"]),
            acceleration_noise_std_g=_nonnegative(imu["acceleration_noise_std_g"], "imu.acceleration_noise_std_g"),
            gyro_noise_std_rad_s=_nonnegative(imu["gyro_noise_std_rad_s"], "imu.gyro_noise_std_rad_s"),
            acceleration_bias_g=_vector3(imu["acceleration_bias_g"], "imu.acceleration_bias_g"),
            gyro_bias_rad_s=_vector3(imu["gyro_bias_rad_s"], "imu.gyro_bias_rad_s"),
            bias_random_walk_std=_nonnegative(imu["bias_random_walk_std"], "imu.bias_random_walk_std"),
        ),
        odometry=OdometryConfig(
            enabled=bool(odometry["enabled"]), rate_hz=_positive(odometry["rate_hz"], "odometry.rate_hz"),
            frame_id=str(odometry["frame_id"]), child_frame_id=str(odometry["child_frame_id"]),
            wheel_scale_error_std=_nonnegative(odometry["wheel_scale_error_std"], "odometry.wheel_scale_error_std"),
            wheel_noise_std_rad_s=_nonnegative(odometry["wheel_noise_std_rad_s"], "odometry.wheel_noise_std_rad_s"),
        ),
        gnss=GnssConfig(
            enabled=bool(gnss["enabled"]), rate_hz=_positive(gnss["rate_hz"], "gnss.rate_hz"),
            frame_id=str(gnss["frame_id"]),
            origin=GnssOriginConfig(float(gnss["origin"]["latitude_deg"]), float(gnss["origin"]["longitude_deg"]), float(gnss["origin"]["altitude_m"])),
            position_noise_std_m=_nonnegative(gnss["position_noise_std_m"], "gnss.position_noise_std_m"),
            velocity_noise_std_mps=_nonnegative(gnss["velocity_noise_std_mps"], "gnss.velocity_noise_std_mps"),
        ),
        runtime=RuntimeConfig(
            headless=bool(runtime["headless"]), fastest=bool(runtime["fastest"]),
            deterministic=bool(runtime["deterministic"]), random_seed=int(runtime["random_seed"]),
            output_failure_limit=_positive_int(runtime["output_failure_limit"], "runtime.output_failure_limit"),
            statistics_interval_s=_positive(runtime["statistics_interval_s"], "runtime.statistics_interval_s"),
        ),
        rerun=RerunConfig(
            spawn=bool(rerun["spawn"]), connect_url=str(rerun["connect_url"]),
            rate_window_s=_positive(rerun["rate_window_s"], "rerun.rate_window_s"),
            statistics_interval_s=_positive(rerun["statistics_interval_s"], "rerun.statistics_interval_s"),
        ),
    )


def load_config(path: Path | None = None, environ: Mapping[str, str] | None = None) -> SimulationConfig:
    """Load defaults, apply one strict YAML override, then environment flags."""
    env = os.environ if environ is None else environ
    selected = path
    if selected is None and env.get("MUJOCO_CONFIG"):
        selected = Path(env["MUJOCO_CONFIG"])
    raw = _load_yaml(DEFAULT_CONFIG_PATH)
    if selected is not None:
        raw = _deep_merge(raw, _load_yaml(Path(selected)))
    if "MUJOCO_HEADLESS" in env:
        raw["runtime"]["headless"] = _bool_env(env["MUJOCO_HEADLESS"], "MUJOCO_HEADLESS")
    if "MUJOCO_FASTEST" in env:
        raw["runtime"]["fastest"] = _bool_env(env["MUJOCO_FASTEST"], "MUJOCO_FASTEST")
    return _build_config(raw)
