from __future__ import annotations

from pathlib import Path
import sys

from scripts.urdf_converter import convert_urdf


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))


MODULE_ROOT = Path(__file__).resolve().parents[1]


def build_test_model(output_dir: Path) -> Path:
    return convert_urdf(
        MODULE_ROOT / "models/mickrobot_3d.urdf",
        MODULE_ROOT / "models/mujoco_extensions.xml",
        MODULE_ROOT / "models/test_world.xml",
        output_dir / "mickrobot.xml",
    )
