import numpy as np
import sys
import os
import time
from .stats import StatsTracker

# wire C++ module
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
try:
    import ourrosbag_cpp
    CPP_AVAILABLE = True
except ImportError:
    CPP_AVAILABLE = False
    print("[Parser] WARNING: C++ module not available, falling back to pure Python")

from .config import BagConfig


class MessageParser:
    def __init__(self, config: BagConfig):
        self.format = config.output.format
        self.stats = StatsTracker()
        print(f"[Parser] C++ hot path: {'enabled' if CPP_AVAILABLE else 'disabled'}")

    def parse(self, topic: str, msgtype: str, timestamp: int, msg) -> dict:
        parsers = {
            "sensor_msgs/msg/Image":       self._parse_image,
            "sensor_msgs/msg/Imu":         self._parse_imu,
            "sensor_msgs/msg/NavSatFix":   self._parse_gps,
            "nav_msgs/msg/Odometry":       self._parse_odometry,
            "sensor_msgs/msg/PointCloud2": self._parse_pointcloud,
        }
        parser_fn = parsers.get(msgtype, self._parse_unknown)

        t0 = time.perf_counter_ns()
        data = parser_fn(msg)
        t1 = time.perf_counter_ns()

        byte_size = len(str(data))
        self.stats.record(msgtype, byte_size, t1 - t0)

        return {
            "topic":     topic,
            "msgtype":   msgtype,
            "timestamp": timestamp,
            "data":      data,
        }

    def _parse_image(self, msg) -> dict:
        if CPP_AVAILABLE:
            channels = 1 if "mono" in msg.encoding.lower() else 3
            arr = ourrosbag_cpp.decode_image(
                bytes(msg.data),
                msg.height,
                msg.width,
                channels
            )
            return {
                "width":    msg.width,
                "height":   msg.height,
                "encoding": msg.encoding,
                "data":     arr,
                "backend":  "cpp",
            }
        # pure Python fallback
        return {
            "width":    msg.width,
            "height":   msg.height,
            "encoding": msg.encoding,
            "data":     np.frombuffer(msg.data, dtype=np.uint8),
            "backend":  "python",
        }

    def _parse_imu(self, msg) -> dict:
        return {
            "orientation":         [msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w],
            "angular_velocity":    [msg.angular_velocity.x,
                                    msg.angular_velocity.y,
                                    msg.angular_velocity.z],
            "linear_acceleration": [msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z],
        }

    def _parse_gps(self, msg) -> dict:
        return {
            "latitude":  msg.latitude,
            "longitude": msg.longitude,
            "altitude":  msg.altitude,
            "status":    msg.status.status,
        }

    def _parse_odometry(self, msg) -> dict:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        return {
            "position":    [p.x, p.y, p.z],
            "orientation": [o.x, o.y, o.z, o.w],
        }

    def _parse_pointcloud(self, msg) -> dict:
        if CPP_AVAILABLE:
            # find x y z field offsets from message metadata
            offsets = {f.name: f.offset for f in msg.fields}
            if all(k in offsets for k in ("x", "y", "z")):
                result = ourrosbag_cpp.parse_pointcloud2(
                    bytes(msg.data),
                    msg.width,
                    msg.height,
                    msg.point_step,
                    offsets["x"],
                    offsets["y"],
                    offsets["z"],
                )
                result["backend"] = "cpp"
                return result

        # pure Python fallback
        return {
            "width":   msg.width,
            "height":  msg.height,
            "fields":  [f.name for f in msg.fields],
            "data":    np.frombuffer(msg.data, dtype=np.uint8),
            "backend": "python",
        }

    def _parse_unknown(self, msg) -> dict:
        return {"raw": str(msg)}