"""Tests for ourrosbag.parser — MessageParser and per-type parse methods."""
import unittest
from unittest.mock import MagicMock, patch
from dataclasses import dataclass, field
import numpy as np


# ── mock ROS message objects ──────────────────────────────────────────────────

@dataclass
class MockVector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockQuaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class MockImage:
    width: int = 640
    height: int = 480
    encoding: str = "rgb8"
    data: bytes = field(default_factory=lambda: bytes(640 * 480 * 3))


@dataclass
class MockImu:
    orientation: MockQuaternion = field(default_factory=MockQuaternion)
    angular_velocity: MockVector3 = field(default_factory=MockVector3)
    linear_acceleration: MockVector3 = field(default_factory=lambda: MockVector3(0, 0, 9.81))


@dataclass
class MockGpsStatus:
    status: int = 0


@dataclass
class MockNavSatFix:
    latitude: float = 37.7749
    longitude: float = -122.4194
    altitude: float = 10.0
    status: MockGpsStatus = field(default_factory=MockGpsStatus)


@dataclass
class MockPose:
    position: MockVector3 = field(default_factory=MockVector3)
    orientation: MockQuaternion = field(default_factory=MockQuaternion)


@dataclass
class MockPoseWithCovariance:
    pose: MockPose = field(default_factory=MockPose)


@dataclass
class MockOdometry:
    pose: MockPoseWithCovariance = field(default_factory=MockPoseWithCovariance)


@dataclass
class MockPointField:
    name: str = ""
    offset: int = 0


@dataclass
class MockPointCloud2:
    width: int = 100
    height: int = 1
    point_step: int = 16
    fields: list = field(default_factory=lambda: [
        MockPointField("x", 0),
        MockPointField("y", 4),
        MockPointField("z", 8),
    ])
    data: bytes = field(default_factory=lambda: bytes(100 * 16))


# ── tests ─────────────────────────────────────────────────────────────────────

class TestParseImage(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_returns_correct_keys(self):
        result = self.parser.parse(
            "/cam", "sensor_msgs/msg/Image", 1000000000, MockImage()
        )
        self.assertEqual(result["topic"], "/cam")
        self.assertEqual(result["msgtype"], "sensor_msgs/msg/Image")
        self.assertEqual(result["timestamp"], 1000000000)
        data = result["data"]
        self.assertIn("width", data)
        self.assertIn("height", data)
        self.assertIn("encoding", data)
        self.assertIn("data", data)

    def test_dimensions_match(self):
        msg = MockImage(width=320, height=240, encoding="mono8",
                        data=bytes(320 * 240))
        result = self.parser.parse("/cam", "sensor_msgs/msg/Image", 0, msg)
        self.assertEqual(result["data"]["width"], 320)
        self.assertEqual(result["data"]["height"], 240)
        self.assertEqual(result["data"]["encoding"], "mono8")


class TestParseImu(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_returns_lists(self):
        result = self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        data = result["data"]
        self.assertIsInstance(data["orientation"], list)
        self.assertIsInstance(data["angular_velocity"], list)
        self.assertIsInstance(data["linear_acceleration"], list)

    def test_orientation_length(self):
        result = self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.assertEqual(len(result["data"]["orientation"]), 4)  # xyzw

    def test_acceleration_values(self):
        result = self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.assertAlmostEqual(result["data"]["linear_acceleration"][2], 9.81)


class TestParseGps(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_lat_lon(self):
        result = self.parser.parse(
            "/gps", "sensor_msgs/msg/NavSatFix", 0, MockNavSatFix()
        )
        data = result["data"]
        self.assertAlmostEqual(data["latitude"], 37.7749)
        self.assertAlmostEqual(data["longitude"], -122.4194)
        self.assertAlmostEqual(data["altitude"], 10.0)
        self.assertEqual(data["status"], 0)


class TestParseOdometry(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_position_and_orientation(self):
        result = self.parser.parse(
            "/odom", "nav_msgs/msg/Odometry", 0, MockOdometry()
        )
        data = result["data"]
        self.assertEqual(len(data["position"]), 3)
        self.assertEqual(len(data["orientation"]), 4)


class TestParseUnknown(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_fallback_returns_raw(self):
        result = self.parser.parse("/x", "custom/msg/Foo", 0, "some raw data")
        self.assertIn("raw", result["data"])


class TestStatsIntegration(unittest.TestCase):
    def setUp(self):
        from ourrosbag.config import BagConfig
        from ourrosbag.parser import MessageParser
        self.parser = MessageParser(BagConfig())

    def test_message_count_increments(self):
        self.assertEqual(self.parser.stats.total, 0)
        self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.assertEqual(self.parser.stats.total, 1)
        self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.assertEqual(self.parser.stats.total, 2)

    def test_per_type_count(self):
        self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.parser.parse("/gps", "sensor_msgs/msg/NavSatFix", 0, MockNavSatFix())
        self.parser.parse("/imu", "sensor_msgs/msg/Imu", 0, MockImu())
        self.assertEqual(self.parser.stats.msg_count["sensor_msgs/msg/Imu"], 2)
        self.assertEqual(self.parser.stats.msg_count["sensor_msgs/msg/NavSatFix"], 1)


if __name__ == "__main__":
    unittest.main()
