"""RGB camera rendering with explicit backend failures."""

from __future__ import annotations

import os

import mujoco
import numpy as np

from ..config import CameraConfig
from ..messages import ImageSample
from ..simulator import MujocoSimulator


class RendererError(RuntimeError):
    """Raised when the configured MuJoCo camera cannot render."""


class CameraSensor:
    def __init__(self, config: CameraConfig, simulator: MujocoSimulator) -> None:
        self.config = config
        self._closed = False
        try:
            self._renderer = mujoco.Renderer(simulator.model, height=config.height, width=config.width)
        except Exception as exc:
            backend = os.environ.get("MUJOCO_GL", "default")
            raise RendererError(
                f"camera renderer initialization failed with MUJOCO_GL={backend!r}; "
                "use a working GUI backend or set MUJOCO_GL=egl/osmesa for headless rendering: "
                f"{exc}"
            ) from exc

    def sample(self, simulator: MujocoSimulator, timestamp_s: float) -> ImageSample:
        if self._closed:
            raise RendererError("camera renderer is closed")
        try:
            self._renderer.update_scene(simulator.data, camera="front_camera")
            rgb = self._renderer.render()
        except Exception as exc:
            raise RendererError(f"camera renderer failed to produce a frame: {exc}") from exc
        image = np.array(rgb, dtype=np.uint8, copy=True, order="C")
        if image.shape != (self.config.height, self.config.width, 3):
            raise RendererError(f"camera renderer returned unexpected shape: {image.shape}")
        return ImageSample(timestamp_s, self.config.frame_id, image)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._renderer.close()
