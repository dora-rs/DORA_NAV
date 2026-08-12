"""Load the finished MuJoCo model without invoking offline conversion."""

from __future__ import annotations

from pathlib import Path

import mujoco


MODULE_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = MODULE_ROOT / "models/mickrobot.xml"
BUILD_INSTRUCTION = "python3 scripts/build_model.py"


class ModelLoadError(RuntimeError):
    """Raised when the offline-generated model is unavailable or invalid."""


def load_generated_model(path: Path = MODEL_PATH) -> mujoco.MjModel:
    candidate = Path(path)
    if not candidate.is_file():
        raise ModelLoadError(f"MuJoCo model is missing: {candidate}; run `{BUILD_INSTRUCTION}` first")
    try:
        return mujoco.MjModel.from_xml_path(str(candidate))
    except Exception as exc:
        raise ModelLoadError(f"MuJoCo model is invalid: {candidate}; rerun `{BUILD_INSTRUCTION}`: {exc}") from exc
