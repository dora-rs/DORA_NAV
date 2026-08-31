import yaml
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class PlaybackConfig:
    speed: float = 1.0
    loop: bool = False
    start_time: float = 0.0
    end_time: Optional[float] = None


@dataclass
class OutputConfig:
    format: str = "arrow"
    prefix: str = "rosbag"


@dataclass
class BagConfig:
    bag_path: str = ""
    topics: List[str] = field(default_factory=list)
    playback: PlaybackConfig = field(default_factory=PlaybackConfig)
    output: OutputConfig = field(default_factory=OutputConfig)


def load_config(path: str = "configs/default.yml") -> BagConfig:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    playback = PlaybackConfig(**raw.get("playback", {}))
    output = OutputConfig(**raw.get("output", {}))

    return BagConfig(
        bag_path=raw.get("bag_path", ""),
        topics=raw.get("topics", []),
        playback=playback,
        output=output,
    )