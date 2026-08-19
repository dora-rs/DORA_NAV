#!/usr/bin/env python3
"""Validate and count all simulated sensor messages in a Dora process."""

from __future__ import annotations

import json
import os
from pathlib import Path

from dora import Node

from mujoco_mickrobot.dora_adapter import _arrow_bytes
from mujoco_mickrobot.messages import decode_gnss, decode_image, decode_imu, decode_odometry, decode_pointcloud


DECODERS = {
    "pointcloud": decode_pointcloud,
    "image": decode_image,
    "imu": decode_imu,
    "Odometry": decode_odometry,
    "gnss": decode_gnss,
}


def write_result(path: Path, counts: dict[str, int], errors: list[str]) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"counts": counts, "errors": errors}), encoding="utf-8")
    temporary.replace(path)


def main() -> None:
    output = Path(os.environ["DORA_COLLECTOR_OUTPUT"])
    counts = {name: 0 for name in DECODERS}
    errors: list[str] = []
    node = Node()
    write_result(output, counts, errors)
    for event in node:
        if event["type"] == "STOP":
            write_result(output, counts, errors)
            return
        if event["type"] != "INPUT" or event["id"] not in DECODERS:
            continue
        port = event["id"]
        try:
            DECODERS[port](_arrow_bytes(event["value"]))
        except Exception as exc:
            errors.append(f"{port}: {exc}")
        else:
            counts[port] += 1
        write_result(output, counts, errors)


if __name__ == "__main__":
    main()
