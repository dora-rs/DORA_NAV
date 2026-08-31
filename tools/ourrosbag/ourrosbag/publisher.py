import json
import numpy as np
from .config import BagConfig


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        return super().default(obj)


def to_arrow(parsed: dict):
    import pyarrow as pa

    msgtype = parsed["msgtype"]
    data = parsed["data"]
    ts = parsed["timestamp"]

    if msgtype == "sensor_msgs/msg/Image":
        arr = data.get("data")
        if isinstance(arr, np.ndarray):
            return pa.record_batch({
                "timestamp": pa.array([ts], type=pa.int64()),
                "width":     pa.array([data["width"]], type=pa.int32()),
                "height":    pa.array([data["height"]], type=pa.int32()),
                "encoding":  pa.array([data["encoding"]], type=pa.string()),
                "data":      pa.array([arr.tobytes()], type=pa.large_binary()),
            })

    if msgtype == "sensor_msgs/msg/Imu":
        return pa.record_batch({
            "timestamp":          pa.array([ts], type=pa.int64()),
            "orientation":        pa.array([data["orientation"]], type=pa.list_(pa.float64())),
            "angular_velocity":   pa.array([data["angular_velocity"]], type=pa.list_(pa.float64())),
            "linear_acceleration":pa.array([data["linear_acceleration"]], type=pa.list_(pa.float64())),
        })

    if msgtype == "nav_msgs/msg/Odometry":
        return pa.record_batch({
            "timestamp":   pa.array([ts], type=pa.int64()),
            "position":    pa.array([data["position"]], type=pa.list_(pa.float64())),
            "orientation": pa.array([data["orientation"]], type=pa.list_(pa.float64())),
        })

    if msgtype == "sensor_msgs/msg/PointCloud2":
        x = data.get("x")
        y = data.get("y")
        z = data.get("z")
        if isinstance(x, np.ndarray):
            return pa.record_batch({
                "timestamp": pa.array([ts], type=pa.int64()),
                "x":         pa.array([x.tobytes()], type=pa.large_binary()),
                "y":         pa.array([y.tobytes()], type=pa.large_binary()),
                "z":         pa.array([z.tobytes()], type=pa.large_binary()),
                "n_points":  pa.array([data["n_points"]], type=pa.int32()),
            })

    # fallback for unknown types
    return pa.record_batch({
        "timestamp": pa.array([ts], type=pa.int64()),
        "raw":       pa.array([json.dumps(data, cls=NumpyEncoder)], type=pa.string()),
    })


class Publisher:
    def __init__(self, config: BagConfig):
        self.format = config.output.format
        self.prefix = config.output.prefix
        self._topic_map = {
            "sensor_msgs/msg/Image":       "image",
            "sensor_msgs/msg/Imu":         "imu",
            "sensor_msgs/msg/NavSatFix":   "gps",
            "nav_msgs/msg/Odometry":       "odometry",
            "sensor_msgs/msg/PointCloud2": "pointcloud",
        }
        try:
            from dora import Node
            self._node = Node()
            self._dora_available = True
            print("[Publisher] Running inside dora runtime.")
        except Exception:
            self._node = None
            self._dora_available = False
            print("[Publisher] Dora not available, running in dry-run mode.")

    def send(self, parsed: dict):
        msgtype = parsed["msgtype"]
        output_id = self._topic_map.get(msgtype, "unknown")

        if self.format == "arrow":
            self._send_arrow(output_id, parsed)
        else:
            payload = json.dumps(parsed, cls=NumpyEncoder).encode()
            self._send_stdout(output_id, payload)

    def _send_arrow(self, output_id: str, parsed: dict):
        batch = to_arrow(parsed)
        if batch is None:
            return
        if self._dora_available:
            import pyarrow as pa
            sink = pa.BufferOutputStream()
            writer = pa.ipc.new_stream(sink, batch.schema)
            writer.write_batch(batch)
            writer.close()
            buf = sink.getvalue()
            import pyarrow as pa
            self._node.send_output(output_id, pa.array([buf.to_pybytes()], type=pa.large_binary()))
        else:
            # dry run — just show schema and size
            print(f"[Publisher] [{output_id}] Arrow batch — {batch.num_rows} row, {batch.schema}")

    def _send_stdout(self, output_id: str, payload: bytes):
        print(f"[Publisher] [{output_id}] {payload[:120]}...")