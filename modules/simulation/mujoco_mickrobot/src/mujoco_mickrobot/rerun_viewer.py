"""Dora sensor frequency monitor and Rerun visualizer."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass
import importlib
import logging
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping

import numpy as np

from .config import RerunConfig, load_config
from .dora_adapter import DoraAdapter, InputEvent, StopEvent
from .messages import (
    ImageSample,
    MessageError,
    decode_gnss,
    decode_image,
    decode_imu,
    decode_odometry,
    decode_pointcloud,
)


LOGGER = logging.getLogger(__name__)
SENSOR_PORTS = ("pointcloud", "image", "imu", "Odometry", "gnss")


@dataclass(frozen=True)
class RateStats:
    hz: float
    count: int
    window_samples: int
    last_sample_timestamp_s: float


class RateMonitor:
    def __init__(self, window_s: float) -> None:
        if not math.isfinite(window_s) or window_s <= 0:
            raise ValueError("rate monitor window_s must be finite and positive")
        self.window_s = window_s
        self._received: dict[str, deque[float]] = defaultdict(deque)
        self._count: dict[str, int] = defaultdict(int)
        self._last_sample: dict[str, float] = {}

    def _evict(self, port: str, now_s: float) -> None:
        cutoff = now_s - self.window_s
        timestamps = self._received[port]
        while timestamps and timestamps[0] < cutoff:
            timestamps.popleft()

    def observe(self, port: str, received_at_s: float, sample_timestamp_s: float) -> None:
        self._received[port].append(received_at_s)
        self._count[port] += 1
        self._last_sample[port] = sample_timestamp_s
        self._evict(port, received_at_s)

    def snapshot(self, now_s: float) -> Mapping[str, RateStats]:
        result = {}
        for port in tuple(self._count):
            self._evict(port, now_s)
            timestamps = self._received[port]
            hz = 0.0
            if len(timestamps) >= 2 and timestamps[-1] > timestamps[0]:
                hz = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
            result[port] = RateStats(hz, self._count[port], len(timestamps), self._last_sample[port])
        return result


class RerunSink:
    def __init__(self, config: RerunConfig) -> None:
        try:
            self._rr = importlib.import_module("rerun")
        except ImportError as exc:
            raise RuntimeError("Rerun SDK is required; install with: pip install -e '.[rerun]'") from exc
        self._viewer_client: Any | None = None
        self._closed = False
        self._rr.init("mujoco_mickrobot_sensors", spawn=False)
        if config.connect_url:
            connect = getattr(self._rr, "connect_grpc", None) or getattr(self._rr, "connect", None)
            if connect is None:
                raise RuntimeError("installed Rerun SDK has no supported connect API")
            connect(config.connect_url)
        elif config.spawn:
            try:
                viewer_client = self._rr.experimental.ViewerClient.spawn(detach_process=False)
            except AttributeError as exc:
                raise RuntimeError("Rerun SDK >=0.36 is required for managed viewer shutdown") from exc
            self._viewer_client = viewer_client
            self._rr.connect_grpc(viewer_client.url)

    def close(self) -> None:
        """Disconnect recording and terminate the viewer owned by this sink."""
        if self._closed:
            return
        self._closed = True
        try:
            self._rr.disconnect()
        finally:
            if self._viewer_client is not None:
                self._viewer_client.close()
                self._viewer_client = None

    def log_pointcloud(self, timestamp_s: float, points_xyzi: np.ndarray) -> None:
        self._rr.set_time("sensor_time", timestamp=timestamp_s)
        intensity = np.clip(points_xyzi[:, 3], 0, 255).astype(np.uint8)
        colors = np.column_stack((intensity, intensity, intensity))
        self._rr.log("mickrobot/lidar/points", self._rr.Points3D(points_xyzi[:, :3], colors=colors))

    def log_image(self, sample: ImageSample) -> None:
        self._rr.set_time("sensor_time", timestamp=sample.timestamp_s)
        self._rr.log("mickrobot/camera/rgb", self._rr.Image(sample.rgb))


class SensorListener:
    def __init__(self, sink: Any, monitor: RateMonitor) -> None:
        self.sink = sink
        self.monitor = monitor

    def handle(self, port: str, payload: bytes, received_at_s: float) -> bool:
        try:
            if port == "pointcloud":
                timestamp_s, points = decode_pointcloud(payload)
                self.sink.log_pointcloud(timestamp_s, points)
            elif port == "image":
                sample = decode_image(payload)
                timestamp_s = sample.timestamp_s
                self.sink.log_image(sample)
            elif port == "imu":
                timestamp_s = decode_imu(payload).timestamp_s
            elif port == "Odometry":
                timestamp_s = decode_odometry(payload).timestamp_s
            elif port == "gnss":
                timestamp_s = decode_gnss(payload).timestamp_s
            else:
                LOGGER.warning("ignoring unknown sensor port %s", port)
                return False
        except (MessageError, ValueError, TypeError) as exc:
            LOGGER.error("cannot decode %s payload: %s", port, exc)
            return False
        self.monitor.observe(port, received_at_s, timestamp_s)
        return True


def _print_stats(monitor: RateMonitor, now_s: float) -> None:
    stats = monitor.snapshot(now_s)
    values = []
    for port in SENSOR_PORTS:
        item = stats.get(port)
        if item is None:
            values.append(f"{port}=--")
        else:
            values.append(f"{port}={item.hz:.1f}Hz total={item.count} sample_t={item.last_sample_timestamp_s:.3f}")
    print(" | ".join(values), flush=True)


def run_listener(
    adapter: DoraAdapter,
    listener: SensorListener,
    sink: RerunSink,
    statistics_interval_s: float,
    *,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """Process Dora events and always release the Rerun connection on exit."""
    next_statistics_s = monotonic() + statistics_interval_s
    try:
        while True:
            event = adapter.poll(0.1)
            now_s = monotonic()
            if isinstance(event, StopEvent):
                return
            if isinstance(event, InputEvent):
                listener.handle(event.input_id, event.payload, now_s)
            if now_s >= next_statistics_s:
                _print_stats(listener.monitor, now_s)
                next_statistics_s = now_s + statistics_interval_s
    finally:
        sink.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor MickRobot Dora sensors and visualize with Rerun")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = load_config(args.config)
    try:
        import dora

        adapter = DoraAdapter(dora.Node())
        sink = RerunSink(config.rerun)
        listener = SensorListener(sink, RateMonitor(config.rerun.rate_window_s))
    except Exception as exc:
        raise SystemExit(f"cannot start Rerun sensor listener: {exc}") from exc
    run_listener(adapter, listener, sink, config.rerun.statistics_interval_s)


if __name__ == "__main__":
    main()
