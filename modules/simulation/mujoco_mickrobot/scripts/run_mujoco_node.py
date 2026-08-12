#!/usr/bin/env python3
"""Run the MuJoCo Dora node directly from the source tree."""

from __future__ import annotations

from pathlib import Path
import sys


MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT / "src"))

from mujoco_mickrobot.main import main  # noqa: E402


if __name__ == "__main__":
    main()
