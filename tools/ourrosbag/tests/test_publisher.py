"""Tests for ourrosbag.publisher — Arrow conversion and dry-run mode."""
import unittest
import numpy as np


class TestToArrowImage(unittest.TestCase):
    def test_image_batch(self):
        from ourrosbag.publisher import to_arrow
        parsed = {
            "msgtype": "sensor_msgs/msg/Image",
            "timestamp": 1000000000,
            "data": {
                "width": 4,
                "height": 2,
                "encoding": "rgb8",
                "data": np.zeros((2, 4, 3), dtype=np.uint8),
            },
        }
        batch = to_arrow(parsed)
        self.assertIsNotNone(batch)
        self.assertEqual(batch.num_rows, 1)
        self.assertIn("width", batch.schema.names)
        self.assertIn("height", batch.schema.names)
        self.assertIn("encoding", batch.schema.names)
        self.assertIn("data", batch.schema.names)


class TestToArrowImu(unittest.TestCase):
    def test_imu_batch(self):
        from ourrosbag.publisher import to_arrow
        parsed = {
            "msgtype": "sensor_msgs/msg/Imu",
            "timestamp": 1000000000,
            "data": {
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "angular_velocity": [0.1, 0.2, 0.3],
                "linear_acceleration": [0.0, 0.0, 9.81],
            },
        }
        batch = to_arrow(parsed)
        self.assertIsNotNone(batch)
        self.assertEqual(batch.num_rows, 1)
        self.assertIn("orientation", batch.schema.names)


class TestToArrowOdometry(unittest.TestCase):
    def test_odometry_batch(self):
        from ourrosbag.publisher import to_arrow
        parsed = {
            "msgtype": "nav_msgs/msg/Odometry",
            "timestamp": 1000000000,
            "data": {
                "position": [1.0, 2.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            },
        }
        batch = to_arrow(parsed)
        self.assertIsNotNone(batch)
        self.assertEqual(batch.num_rows, 1)
        self.assertIn("position", batch.schema.names)


class TestToArrowPointCloud(unittest.TestCase):
    def test_pointcloud_batch(self):
        from ourrosbag.publisher import to_arrow
        n = 10
        parsed = {
            "msgtype": "sensor_msgs/msg/PointCloud2",
            "timestamp": 1000000000,
            "data": {
                "x": np.zeros(n, dtype=np.float32),
                "y": np.ones(n, dtype=np.float32),
                "z": np.full(n, 2.0, dtype=np.float32),
                "n_points": n,
            },
        }
        batch = to_arrow(parsed)
        self.assertIsNotNone(batch)
        self.assertIn("n_points", batch.schema.names)


class TestToArrowUnknown(unittest.TestCase):
    def test_fallback_returns_raw(self):
        from ourrosbag.publisher import to_arrow
        parsed = {
            "msgtype": "custom/msg/Foo",
            "timestamp": 1000000000,
            "data": {"key": "value"},
        }
        batch = to_arrow(parsed)
        self.assertIsNotNone(batch)
        self.assertIn("raw", batch.schema.names)
        self.assertEqual(batch.num_rows, 1)


class TestPublisherDryRun(unittest.TestCase):
    def test_dry_run_does_not_crash(self):
        """Publisher without dora should run in dry-run mode without errors."""
        from ourrosbag.config import BagConfig
        from ourrosbag.publisher import Publisher
        pub = Publisher(BagConfig())
        self.assertFalse(pub._dora_available)

        parsed = {
            "topic": "/imu",
            "msgtype": "sensor_msgs/msg/Imu",
            "timestamp": 1000000000,
            "data": {
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "angular_velocity": [0.1, 0.2, 0.3],
                "linear_acceleration": [0.0, 0.0, 9.81],
            },
        }
        # should print to stdout, not raise
        pub.send(parsed)


class TestNumpyEncoder(unittest.TestCase):
    def test_encodes_ndarray(self):
        import json
        from ourrosbag.publisher import NumpyEncoder
        arr = np.array([1.0, 2.0, 3.0])
        result = json.dumps({"data": arr}, cls=NumpyEncoder)
        self.assertIn("[1.0, 2.0, 3.0]", result)

    def test_encodes_np_scalar(self):
        import json
        from ourrosbag.publisher import NumpyEncoder
        val = np.float64(3.14)
        result = json.dumps({"val": val}, cls=NumpyEncoder)
        self.assertIn("3.14", result)


if __name__ == "__main__":
    unittest.main()
