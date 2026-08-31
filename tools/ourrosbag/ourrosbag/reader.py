from pathlib import Path
from .config import BagConfig


def _is_v3(path: Path) -> bool:
    # v3 (ROS2) bags are directories containing metadata.yaml
    # v2 (ROS1) bags are single .bag files
    if path.is_dir():
        return (path / "metadata.yaml").exists()
    return False


class BagReader:
    def __init__(self, config: BagConfig):
        self.path = Path(config.bag_path)
        self.topics = config.topics
        self.version = self._detect_version()
        self._validate()
        print(f"[BagReader] Detected ROSbag {'v3 (ROS2)' if self.version == 3 else 'v2 (ROS1)'}")

    def _detect_version(self) -> int:
        if _is_v3(self.path):
            return 3
        if self.path.suffix == ".bag":
            return 2
        # if path has no extension, check if it's a v3 directory
        if self.path.is_dir():
            return 3
        return 2

    def _validate(self):
        if not self.path.exists():
            raise FileNotFoundError(f"Bag not found: {self.path}")
        if self.version == 2 and self.path.suffix not in (".bag",):
            raise ValueError(f"Not a .bag file: {self.path}")

    def _get_typestore(self):
        from rosbags.typesys import Stores, get_typestore
        if self.version == 3:
            return get_typestore(Stores.ROS2_HUMBLE)
        return get_typestore(Stores.ROS1_NOETIC)

    def get_available_topics(self) -> dict:
        if self.version == 3:
            from rosbags.rosbag2 import Reader
        else:
            from rosbags.rosbag1 import Reader

        with Reader(self.path) as reader:
            return {
                conn.topic: conn.msgtype
                for conn in reader.connections
            }

    def filter_topics(self, available: dict) -> list:
        if not self.topics:
            return list(available.keys())
        return [t for t in self.topics if t in available]

    def messages(self):
        if self.version == 3:
            from rosbags.rosbag2 import Reader
        else:
            from rosbags.rosbag1 import Reader

        typestore = self._get_typestore()
        available = self.get_available_topics()
        selected = self.filter_topics(available)

        print(f"[BagReader] Available topics: {list(available.keys())}")
        print(f"[BagReader] Selected topics: {selected}")

        with Reader(self.path) as reader:
            connections = [
                c for c in reader.connections
                if c.topic in selected
            ]
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                if self.version == 3:
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                else:
                    msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
                yield conn.topic, conn.msgtype, timestamp, msg