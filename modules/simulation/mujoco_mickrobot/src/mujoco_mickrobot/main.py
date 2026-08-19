"""Command-line entry point for the MuJoCo Dora simulation node."""

from __future__ import annotations

import argparse
from dataclasses import replace
import logging
from pathlib import Path
import signal

from .config import SimulationConfig, load_config
from .dora_adapter import DoraAdapter
from .messages import encode_gnss, encode_image, encode_imu, encode_odometry, encode_pointcloud
from .model_loader import MODEL_PATH
from .runtime import SensorBinding, SimulationRuntime
from .sensors.camera import CameraSensor
from .sensors.gnss import GnssSensor
from .sensors.imu import ImuSensor
from .sensors.lidar import LidarSensor
from .sensors.odometry import WheelOdometry
from .simulator import MujocoSimulator
from .vehicle import VehicleController


def _bindings(config: SimulationConfig, simulator: MujocoSimulator) -> tuple[SensorBinding, ...]:
    deterministic = config.runtime.deterministic
    seed = config.runtime.random_seed
    result: list[SensorBinding] = []
    if config.lidar.enabled:
        lidar = LidarSensor(config.lidar, simulator, deterministic=deterministic, seed=seed + 1)
        result.append(SensorBinding("pointcloud", config.lidar.rate_hz, lambda snapshot, timestamp: encode_pointcloud(timestamp, lidar.sample(simulator, timestamp)), True))
    if config.camera.enabled:
        camera = CameraSensor(config.camera, simulator)
        result.append(SensorBinding("image", config.camera.rate_hz, lambda snapshot, timestamp: encode_image(camera.sample(simulator, timestamp)), True, camera.close))
    if config.imu.enabled:
        imu = ImuSensor(config.imu, deterministic=deterministic, seed=seed + 2)
        result.append(SensorBinding("imu", config.imu.rate_hz, lambda snapshot, timestamp: encode_imu(imu.sample(snapshot, timestamp)), False))
    if config.odometry.enabled:
        odometry = WheelOdometry(config.odometry, config.vehicle, deterministic=deterministic, seed=seed + 3)
        result.append(SensorBinding("Odometry", config.odometry.rate_hz, lambda snapshot, timestamp: encode_odometry(odometry.sample(snapshot, timestamp)), False))
    if config.gnss.enabled:
        gnss = GnssSensor(config.gnss, deterministic=deterministic, seed=seed + 4)
        result.append(SensorBinding("gnss", config.gnss.rate_hz, lambda snapshot, timestamp: encode_gnss(gnss.sample(snapshot, timestamp)), False))
    return tuple(result)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MickRobot in MuJoCo as a Dora node")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fastest", action="store_true")
    parser.add_argument("--validate-model", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = load_config(args.config)
    if args.headless or args.fastest:
        config = replace(config, runtime=replace(config.runtime, headless=args.headless or config.runtime.headless, fastest=args.fastest or config.runtime.fastest))
    simulator = MujocoSimulator(config, MODEL_PATH)
    if args.validate_model:
        print(f"valid model: {MODEL_PATH} nq={simulator.model.nq} nv={simulator.model.nv} nu={simulator.model.nu}")
        simulator.close()
        return
    if not config.runtime.headless:
        simulator.launch_viewer()
    try:
        import dora

        adapter = DoraAdapter(dora.Node())
        runtime = SimulationRuntime(config, simulator, adapter, VehicleController(config.vehicle), _bindings(config, simulator))
    except Exception:
        simulator.close()
        raise
    signal.signal(signal.SIGINT, lambda signum, frame: runtime.request_stop())
    signal.signal(signal.SIGTERM, lambda signum, frame: runtime.request_stop())
    raise SystemExit(runtime.run())


if __name__ == "__main__":
    main()
