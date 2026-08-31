import pyarrow as pa
import pyarrow.parquet as pq
import numpy as np
from pathlib import Path


class Recorder:
    """
    Records parsed dora messages into per-topic Parquet files using
    Arrow RecordBatches. Buffers rows in memory and flushes periodically
    to keep memory bounded on large bags.
    """

    FLUSH_EVERY = 500

    def __init__(self, output_dir: str = "output/parquet_session"):
        self.dir = Path(output_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._writers = {}      # msgtype -> ParquetWriter
        self._buffers = {}      # msgtype -> list of row dicts
        self._schemas = {}      # msgtype -> pa.Schema
        self._count = 0
        print(f"[Recorder] Writing Parquet to {self.dir}/")

    def _topic_filename(self, msgtype: str) -> str:
        short = msgtype.split("/")[-1].lower()
        return str(self.dir / f"{short}.parquet")

    def _row_for(self, parsed: dict) -> dict:
        msgtype = parsed["msgtype"]
        data = parsed["data"]
        base = {"topic": parsed["topic"], "timestamp": parsed["timestamp"]}

        if msgtype == "sensor_msgs/msg/Image":
            arr = data.get("data")
            raw_bytes = arr.tobytes() if isinstance(arr, np.ndarray) else bytes(arr)
            base.update({
                "width": data["width"],
                "height": data["height"],
                "encoding": data["encoding"],
                "pixels": raw_bytes,
            })

        elif msgtype == "sensor_msgs/msg/Imu":
            base.update({
                "orientation": data["orientation"],
                "angular_velocity": data["angular_velocity"],
                "linear_acceleration": data["linear_acceleration"],
            })

        elif msgtype == "nav_msgs/msg/Odometry":
            base.update({
                "position": data["position"],
                "orientation": data["orientation"],
            })

        elif msgtype == "sensor_msgs/msg/NavSatFix":
            base.update({
                "latitude": data["latitude"],
                "longitude": data["longitude"],
                "altitude": data["altitude"],
                "status": data["status"],
            })

        elif msgtype == "sensor_msgs/msg/PointCloud2":
            x = data.get("x")
            y = data.get("y")
            z = data.get("z")
            base.update({
                "n_points": data.get("n_points", 0),
                "x_bytes": x.tobytes() if isinstance(x, np.ndarray) else b"",
                "y_bytes": y.tobytes() if isinstance(y, np.ndarray) else b"",
                "z_bytes": z.tobytes() if isinstance(z, np.ndarray) else b"",
            })

        return base

    def record(self, parsed: dict):
        msgtype = parsed["msgtype"]
        row = self._row_for(parsed)

        self._buffers.setdefault(msgtype, []).append(row)
        self._count += 1

        if len(self._buffers[msgtype]) >= self.FLUSH_EVERY:
            self._flush(msgtype)

    def _flush(self, msgtype: str):
        rows = self._buffers.get(msgtype)
        if not rows:
            return

        table = pa.Table.from_pylist(rows)

        if msgtype not in self._writers:
            path = self._topic_filename(msgtype)
            self._writers[msgtype] = pq.ParquetWriter(
                path, table.schema, compression="zstd"
            )
            self._schemas[msgtype] = table.schema

        # schema must match exactly across batches
        table = table.cast(self._schemas[msgtype])
        self._writers[msgtype].write_table(table)
        self._buffers[msgtype] = []

    def close(self):
        for msgtype in list(self._buffers.keys()):
            self._flush(msgtype)
        for writer in self._writers.values():
            writer.close()
        print(f"[Recorder] Saved {self._count} messages across {len(self._writers)} Parquet files to {self.dir}/")
        for msgtype, path in [(m, self._topic_filename(m)) for m in self._writers]:
            size_mb = Path(path).stat().st_size / 1e6
            print(f"  {Path(path).name:25s} {size_mb:.2f} MB")