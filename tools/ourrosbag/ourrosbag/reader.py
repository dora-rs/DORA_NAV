from pathlib import Path
from .config import BagConfig


def _is_v3(path: Path) -> bool:
    """v3 (ROS2) bags are directories containing metadata.yaml.
    v2 (ROS1) bags are single .bag files."""
    if path.is_dir():
        return (path / "metadata.yaml").exists()
    return False


def _read_v3_metadata(path: Path) -> dict:
    """Parse metadata.yaml from a ROS 2 bag directory.
    Returns a dict with storage_plugin, duration_ns, and message_count,
    or empty dict if parsing fails."""
    meta_path = path / "metadata.yaml"
    if not meta_path.exists():
        return {}
    try:
        import yaml
        with open(meta_path, "r") as f:
            raw = yaml.safe_load(f)
        info = raw.get("rosbag2_bagfile_information", {})
        if not info:
            return {}

        storage = info.get("storage_identifier", "unknown")
        duration = info.get("duration", {})
        # duration can be a dict with nanoseconds key, or a plain int
        if isinstance(duration, dict):
            duration_ns = duration.get("nanoseconds", 0)
        else:
            duration_ns = int(duration) if duration else 0
        message_count = info.get("message_count", 0)

        return {
            "storage_plugin": storage,
            "duration_ns": duration_ns,
            "message_count": message_count,
        }
    except Exception:
        return {}


class BagReader:
    def __init__(self, config: BagConfig):
        self.path = Path(config.bag_path)
        self.topics = config.topics
        self.version = self._detect_version()
        self._validate()
        self._print_info()

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
        if self.version == 3:
            meta_path = self.path / "metadata.yaml"
            if not meta_path.exists():
                raise FileNotFoundError(
                    f"Directory '{self.path}' looks like a ROS 2 bag but is missing "
                    f"metadata.yaml. Ensure this is a valid rosbag2 directory."
                )
            try:
                import yaml
                with open(meta_path, "r") as f:
                    raw = yaml.safe_load(f)
                if not raw or "rosbag2_bagfile_information" not in raw:
                    raise ValueError(
                        f"metadata.yaml in '{self.path}' does not contain "
                        f"'rosbag2_bagfile_information'. Is this a valid ROS 2 bag?"
                    )
            except ImportError:
                pass  # yaml not available, skip deep validation

    def _print_info(self):
        if self.version == 3:
            meta = _read_v3_metadata(self.path)
            storage = meta.get("storage_plugin", "unknown")
            duration_ns = meta.get("duration_ns", 0)
            message_count = meta.get("message_count", 0)
            duration_s = duration_ns / 1e9 if duration_ns else 0
            print(f"[BagReader] Detected ROSbag v3 (ROS2)")
            print(f"[BagReader]   storage : {storage}")
            if duration_s > 0:
                print(f"[BagReader]   duration: {duration_s:.2f}s")
            if message_count > 0:
                print(f"[BagReader]   messages: {message_count}")
        else:
            print(f"[BagReader] Detected ROSbag v2 (ROS1)")

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