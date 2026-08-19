"""Narrow Dora/PyArrow byte boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pyarrow as pa


class DoraAdapterError(RuntimeError):
    """Raised when a Dora event cannot satisfy the byte contract."""


@dataclass(frozen=True)
class InputEvent:
    input_id: str
    payload: bytes


@dataclass(frozen=True)
class StopEvent:
    pass


def _arrow_bytes(value: Any) -> bytes:
    if isinstance(value, pa.ChunkedArray):
        value = value.combine_chunks()
    if isinstance(value, pa.Array):
        if pa.types.is_binary(value.type) or pa.types.is_large_binary(value.type) or pa.types.is_string(value.type):
            if len(value) != 1:
                raise DoraAdapterError("binary Dora input must contain exactly one Arrow value")
            item = value[0].as_py()
            return item.encode("utf-8") if isinstance(item, str) else bytes(item)
        return value.to_numpy(zero_copy_only=False).tobytes()
    try:
        return bytes(value)
    except (TypeError, ValueError) as exc:
        raise DoraAdapterError(f"Dora input cannot be converted to bytes: {exc}") from exc


class DoraAdapter:
    def __init__(self, node: Any) -> None:
        self._node = node

    def poll(self, timeout_s: float) -> InputEvent | StopEvent | None:
        event = self._node.next(timeout=timeout_s)
        if event is None:
            return None
        event_type = event.get("type")
        if event_type == "STOP":
            return StopEvent()
        if event_type != "INPUT":
            return None
        if "id" not in event or "value" not in event:
            raise DoraAdapterError("Dora INPUT event requires id and value")
        return InputEvent(str(event["id"]), _arrow_bytes(event["value"]))

    def send(self, output_id: str, payload: bytes, timestamp_s: float) -> None:
        values = pa.array(np.frombuffer(payload, dtype=np.uint8), type=pa.uint8())
        self._node.send_output(output_id, values, {"timestamp": float(timestamp_s)})
