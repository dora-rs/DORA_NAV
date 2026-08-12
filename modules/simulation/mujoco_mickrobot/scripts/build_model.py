#!/usr/bin/env python3
"""Offline entry point for MickRobot URDF to MuJoCo conversion."""

from pathlib import Path

import mujoco

from urdf_converter import convert_urdf


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    models_dir = root / "models"
    output = convert_urdf(
        models_dir / "mickrobot_3d.urdf",
        models_dir / "mujoco_extensions.xml",
        models_dir / "test_world.xml",
        models_dir / "mickrobot.xml",
    )
    model = mujoco.MjModel.from_xml_path(str(output))
    print(f"model={output} nq={model.nq} nv={model.nv} nu={model.nu}")


if __name__ == "__main__":
    main()
