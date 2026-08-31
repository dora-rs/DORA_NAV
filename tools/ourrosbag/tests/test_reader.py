"""Tests for ourrosbag.reader — version detection, validation, topic filtering."""
import unittest
from unittest.mock import patch, MagicMock
from pathlib import Path
import tempfile
import os


class TestIsV3(unittest.TestCase):
    def test_directory_with_metadata(self):
        from ourrosbag.reader import _is_v3
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
            self.assertTrue(_is_v3(Path(d)))

    def test_directory_without_metadata(self):
        from ourrosbag.reader import _is_v3
        with tempfile.TemporaryDirectory() as d:
            self.assertFalse(_is_v3(Path(d)))

    def test_file_path(self):
        from ourrosbag.reader import _is_v3
        with tempfile.NamedTemporaryFile(suffix=".bag", delete=False) as f:
            f.write(b"dummy")
            path = f.name
        try:
            self.assertFalse(_is_v3(Path(path)))
        finally:
            os.unlink(path)


class TestReadV3Metadata(unittest.TestCase):
    def test_parses_valid_metadata(self):
        from ourrosbag.reader import _read_v3_metadata
        with tempfile.TemporaryDirectory() as d:
            meta = Path(d) / "metadata.yaml"
            meta.write_text(
                "rosbag2_bagfile_information:\n"
                "  storage_identifier: mcap\n"
                "  duration:\n"
                "    nanoseconds: 5000000000\n"
                "  message_count: 1234\n"
            )
            result = _read_v3_metadata(Path(d))
            self.assertEqual(result["storage_plugin"], "mcap")
            self.assertEqual(result["duration_ns"], 5000000000)
            self.assertEqual(result["message_count"], 1234)

    def test_returns_empty_for_missing_file(self):
        from ourrosbag.reader import _read_v3_metadata
        with tempfile.TemporaryDirectory() as d:
            result = _read_v3_metadata(Path(d))
            self.assertEqual(result, {})

    def test_returns_empty_for_bad_yaml(self):
        from ourrosbag.reader import _read_v3_metadata
        with tempfile.TemporaryDirectory() as d:
            meta = Path(d) / "metadata.yaml"
            meta.write_text("not_the_right_key: 42\n")
            result = _read_v3_metadata(Path(d))
            self.assertEqual(result, {})

    def test_handles_plain_int_duration(self):
        from ourrosbag.reader import _read_v3_metadata
        with tempfile.TemporaryDirectory() as d:
            meta = Path(d) / "metadata.yaml"
            meta.write_text(
                "rosbag2_bagfile_information:\n"
                "  storage_identifier: sqlite3\n"
                "  duration: 3000000000\n"
                "  message_count: 500\n"
            )
            result = _read_v3_metadata(Path(d))
            self.assertEqual(result["duration_ns"], 3000000000)


class TestDetectVersion(unittest.TestCase):
    def test_bag_file_is_v2(self):
        with tempfile.NamedTemporaryFile(suffix=".bag", delete=False) as f:
            f.write(b"dummy")
            path = f.name
        try:
            from ourrosbag.reader import BagReader
            from ourrosbag.config import BagConfig
            config = BagConfig(bag_path=path)
            # bypass full init — just test _detect_version
            reader = object.__new__(BagReader)
            reader.path = Path(path)
            reader.topics = []
            self.assertEqual(reader._detect_version(), 2)
        finally:
            os.unlink(path)

    def test_directory_with_metadata_is_v3(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n"
            )
            from ourrosbag.reader import BagReader
            reader = object.__new__(BagReader)
            reader.path = Path(d)
            reader.topics = []
            self.assertEqual(reader._detect_version(), 3)


class TestValidation(unittest.TestCase):
    def test_missing_path_raises(self):
        from ourrosbag.reader import BagReader
        from ourrosbag.config import BagConfig
        config = BagConfig(bag_path="/nonexistent/path/fake.bag")
        with self.assertRaises(FileNotFoundError):
            BagReader(config)

    def test_wrong_extension_raises(self):
        with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
            f.write(b"dummy")
            path = f.name
        try:
            from ourrosbag.reader import BagReader
            from ourrosbag.config import BagConfig
            config = BagConfig(bag_path=path)
            with self.assertRaises(ValueError):
                BagReader(config)
        finally:
            os.unlink(path)

    def test_v3_dir_without_metadata_raises(self):
        with tempfile.TemporaryDirectory() as d:
            # create a subdirectory that looks like a bag dir but has no metadata
            bag_dir = Path(d) / "mybag"
            bag_dir.mkdir()
            from ourrosbag.reader import BagReader
            from ourrosbag.config import BagConfig
            config = BagConfig(bag_path=str(bag_dir))
            with self.assertRaises(FileNotFoundError):
                BagReader(config)


class TestFilterTopics(unittest.TestCase):
    def setUp(self):
        from ourrosbag.reader import BagReader
        self.reader = object.__new__(BagReader)
        self.reader.path = Path(".")
        self.available = {
            "/camera/image": "sensor_msgs/msg/Image",
            "/imu/data": "sensor_msgs/msg/Imu",
            "/odom": "nav_msgs/msg/Odometry",
        }

    def test_empty_filter_returns_all(self):
        self.reader.topics = []
        result = self.reader.filter_topics(self.available)
        self.assertEqual(len(result), 3)

    def test_partial_match(self):
        self.reader.topics = ["/camera/image", "/nonexistent"]
        result = self.reader.filter_topics(self.available)
        self.assertEqual(result, ["/camera/image"])

    def test_full_match(self):
        self.reader.topics = ["/camera/image", "/imu/data"]
        result = self.reader.filter_topics(self.available)
        self.assertEqual(len(result), 2)


if __name__ == "__main__":
    unittest.main()
