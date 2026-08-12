"""Simulation/Dora orchestration and safe shutdown."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import time
from typing import Callable, Protocol

from .config import SimulationConfig
from .dora_adapter import InputEvent, StopEvent
from .scheduler import PeriodicSchedule
from .simulator import SimulationSnapshot
from .vehicle import CommandError, VehicleController, WheelTargets, parse_cmd_vel


LOGGER = logging.getLogger(__name__)


class SimulatorProtocol(Protocol):
    time_s: float

    def step(self, targets: WheelTargets) -> None: ...
    def snapshot(self) -> SimulationSnapshot: ...
    def viewer_is_running(self) -> bool: ...
    def sync_viewer(self) -> None: ...
    def close(self) -> None: ...


class AdapterProtocol(Protocol):
    def poll(self, timeout_s: float): ...
    def send(self, output_id: str, payload: bytes, timestamp_s: float) -> None: ...


@dataclass(frozen=True)
class SensorBinding:
    output_id: str
    rate_hz: float
    sample_and_encode: Callable[[SimulationSnapshot, float], bytes]
    expensive: bool
    close: Callable[[], None] | None = None


class RuntimeFailure(RuntimeError):
    """Raised after a configured number of consecutive output failures."""


class SimulationRuntime:
    def __init__(
        self,
        config: SimulationConfig,
        simulator: SimulatorProtocol,
        adapter: AdapterProtocol,
        controller: VehicleController,
        bindings: tuple[SensorBinding, ...],
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.simulator = simulator
        self.adapter = adapter
        self.controller = controller
        self.bindings = bindings
        self._schedules = {binding.output_id: PeriodicSchedule(binding.rate_hz) for binding in bindings}
        self._failures = {binding.output_id: 0 for binding in bindings}
        self._monotonic = monotonic
        self._sleep = sleep
        self._stop_requested = False
        self._closed = False
        self._wall_started_s = monotonic()

    def request_stop(self) -> None:
        self._stop_requested = True

    def _drain_events(self) -> None:
        while True:
            event = self.adapter.poll(0.0)
            if event is None:
                return
            if isinstance(event, StopEvent):
                self.request_stop()
                return
            if isinstance(event, InputEvent) and event.input_id == "cmd_vel":
                try:
                    command = parse_cmd_vel(event.payload)
                except CommandError as exc:
                    LOGGER.warning("ignoring invalid cmd_vel: %s", exc)
                    continue
                self.controller.apply(command, self._monotonic())

    def _publish_due(self) -> None:
        snapshot = self.simulator.snapshot()
        for binding in self.bindings:
            due = self._schedules[binding.output_id].due(self.simulator.time_s)
            if binding.expensive and due:
                due = (due[-1],)
            for timestamp_s in due:
                payload = binding.sample_and_encode(snapshot, timestamp_s)
                try:
                    self.adapter.send(binding.output_id, payload, timestamp_s)
                except Exception as exc:
                    self._failures[binding.output_id] += 1
                    LOGGER.error("Dora output %s failed (%d/%d): %s", binding.output_id, self._failures[binding.output_id], self.config.runtime.output_failure_limit, exc)
                    if self._failures[binding.output_id] >= self.config.runtime.output_failure_limit:
                        raise RuntimeFailure(f"Dora output {binding.output_id} failed repeatedly") from exc
                else:
                    self._failures[binding.output_id] = 0

    def _throttle(self) -> None:
        if self.config.runtime.fastest:
            return
        target = self._wall_started_s + self.simulator.time_s / self.config.physics.realtime_factor
        delay = target - self._monotonic()
        if delay > 0:
            self._sleep(delay)

    def _iteration(self) -> None:
        self._drain_events()
        if self._stop_requested:
            return
        targets = self.controller.update(self._monotonic(), self.config.physics.timestep_s)
        self.simulator.step(targets)
        self._publish_due()
        self.simulator.sync_viewer()
        if not self.simulator.viewer_is_running():
            self.request_stop()
        self._throttle()

    def run_until(self, end_simulation_s: float) -> None:
        while self.simulator.time_s < end_simulation_s and not self._stop_requested:
            self._iteration()

    def run(self) -> int:
        result = 0
        try:
            while not self._stop_requested:
                self._iteration()
        except RuntimeFailure as exc:
            LOGGER.error("simulation runtime stopped: %s", exc)
            result = 1
        finally:
            self.close()
        return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.simulator.step(self.controller.stop())
        finally:
            for binding in self.bindings:
                if binding.close is not None:
                    binding.close()
            self.simulator.close()
