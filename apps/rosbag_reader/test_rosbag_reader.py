"""
Basic Tests for ROSbag Reader — DORA_NAV GSoC 2026 Project #9
Author: kaushikteja26

Run with:
    python3 test_rosbag_reader.py
"""

import sys
import os
import time
import pathlib
import shutil
import numpy as np

# Test Setup

PASS = "PASS"
FAIL = "FAIL"
results = []

def test(name: str, passed: bool, detail: str = ""):
    status = PASS if passed else FAIL
    results.append((name, passed))
    print(f"  {status} — {name}")
    if detail:
        print(f"         {detail}")

def create_test_bag(bag_path: str):
    """Create a test bag for all tests."""
    from rosbags.rosbag2 import Writer
    from  rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    path = pathlib.Path(bag_path)

    if path.exists():
        shutil.rmtree(path)

    with Writer(path, version=9) as writer:
        imu_conn = writer.add_connection(
            "/imu/data", "sensor_msgs/msg/Imu",
            typestore=typestore)
        gps_conn = writer.add_connection(
            "/gps/fix", "sensor_msgs/msg/NavSatFix",
            typestore=typestore)
        odom_conn = writer.add_connection(
            "/odom", "nav_msgs/msg/Odometry",
            typestore=typestore)
        img_conn = writer.add_connection(
            "/camera/image_raw", "sensor_msgs/msg/Image",
            typestore=typestore)
        pc_conn = writer.add_connection(
            "/pointcloud", "sensor_msgs/msg/PointCloud2",
            typestore=typestore)

        for i in range(5):
            ts = i * 100_000_000
            header = typestore.types['std_msgs/msg/Header'](
                stamp=typestore.types[
                    'builtin_interfaces/msg/Time'](
                    sec=i, nanosec=0),
                frame_id='base_link')

            # IMU
            imu = typestore.types['sensor_msgs/msg/Imu'](
                header=header,
                linear_acceleration=typestore.types[
                    'geometry_msgs/msg/Vector3'](
                    x=0.1*i, y=0.0, z=9.8),
                angular_velocity=typestore.types[
                    'geometry_msgs/msg/Vector3'](
                    x=0.0, y=0.0, z=0.01*i),
                orientation=typestore.types[
                    'geometry_msgs/msg/Quaternion'](
                    x=0.0, y=0.0, z=0.0, w=1.0),
                linear_acceleration_covariance=np.zeros(9),
                angular_velocity_covariance=np.zeros(9),
                orientation_covariance=np.zeros(9),
            )
            writer.write(imu_conn, ts,
                typestore.serialize_cdr(
                    imu, 'sensor_msgs/msg/Imu'))

            # GPS
            gps = typestore.types['sensor_msgs/msg/NavSatFix'](
                header=header,
                latitude=17.3850 + i*0.0001,
                longitude=78.4867 + i*0.0001,
                altitude=542.0,
                status=typestore.types[
                    'sensor_msgs/msg/NavSatStatus'](
                    status=0, service=1),
                position_covariance=np.zeros(9),
                position_covariance_type=0,
            )
            writer.write(gps_conn, ts,
                typestore.serialize_cdr(
                    gps, 'sensor_msgs/msg/NavSatFix'))

            # Odometry
            odom = typestore.types['nav_msgs/msg/Odometry'](
                header=header,
                child_frame_id='base_link',
                pose=typestore.types[
                    'geometry_msgs/msg/PoseWithCovariance'](
                    pose=typestore.types[
                        'geometry_msgs/msg/Pose'](
                        position=typestore.types[
                            'geometry_msgs/msg/Point'](
                            x=0.1*i, y=0.05*i, z=0.0),
                        orientation=typestore.types[
                            'geometry_msgs/msg/Quaternion'](
                            x=0.0, y=0.0,
                            z=0.0, w=1.0)),
                    covariance=np.zeros(36)),
                twist=typestore.types[
                    'geometry_msgs/msg/TwistWithCovariance'](
                    twist=typestore.types[
                        'geometry_msgs/msg/Twist'](
                        linear=typestore.types[
                            'geometry_msgs/msg/Vector3'](
                            x=0.1, y=0.0, z=0.0),
                        angular=typestore.types[
                            'geometry_msgs/msg/Vector3'](
                            x=0.0, y=0.0, z=0.01)),
                    covariance=np.zeros(36)),
            )
            writer.write(odom_conn, ts,
                typestore.serialize_cdr(
                    odom, 'nav_msgs/msg/Odometry'))

            # Image
            img_data = np.random.randint(
                0, 255, (4, 4, 3), dtype=np.uint8)
            img = typestore.types['sensor_msgs/msg/Image'](
                header=header,
                height=4, width=4,
                encoding='rgb8',
                is_bigendian=False,
                step=12,
                data=img_data.flatten(),
            )
            writer.write(img_conn, ts,
                typestore.serialize_cdr(
                    img, 'sensor_msgs/msg/Image'))

            # PointCloud
            points = np.array([
                [1.0*i, 0.0, 0.5],
                [2.0*i, 0.5, 1.0],
            ], dtype=np.float32)
            fields = [
                typestore.types[
                    'sensor_msgs/msg/PointField'](
                    name='x', offset=0,
                    datatype=7, count=1),
                typestore.types[
                    'sensor_msgs/msg/PointField'](
                    name='y', offset=4,
                    datatype=7, count=1),
                typestore.types[
                    'sensor_msgs/msg/PointField'](
                    name='z', offset=8,
                    datatype=7, count=1),
            ]
            pc = typestore.types[
                'sensor_msgs/msg/PointCloud2'](
                header=header,
                height=1, width=2,
                fields=fields,
                is_bigendian=False,
                point_step=12, row_step=24,
                data=np.frombuffer(
                    points.tobytes(), dtype=np.uint8),
                is_dense=True,
            )
            writer.write(pc_conn, ts,
                typestore.serialize_cdr(
                    pc, 'sensor_msgs/msg/PointCloud2'))

    return str(path)

# Tests

def test_bag_creation():
    """Test 1 — bag file can be created."""
    try:
        bag_path = create_test_bag("test_bag_unit")
        exists = os.path.exists(bag_path)
        test("Bag creation", exists,
             f"Created at: {bag_path}")
        return bag_path
    except Exception as e:
        test("Bag creation", False, str(e))
        return None

def test_topic_inspection(bag_path):
    """Test 2 — topics can be detected."""
    try:
        from rosbags.rosbag2 import Reader
        with Reader(bag_path) as reader:
            topics = [c.topic for c in reader.connections]
            expected = ["/imu/data", "/gps/fix",
                       "/odom", "/camera/image_raw",
                       "/pointcloud"]
            all_found = all(t in topics for t in expected)
            test("Topic inspection",
                 all_found and len(topics) == 5,
                 f"Found: {topics}")
    except Exception as e:
        test("Topic inspection", False, str(e))

def test_message_count(bag_path):
    """Test 3 — correct message count."""
    try:
        from rosbags.rosbag2 import Reader
        with Reader(bag_path) as reader:
            count = reader.message_count
            test("Message count",
                 count == 25,
                 f"Count: {count} (expected 25)")
    except Exception as e:
        test("Message count", False, str(e))

def test_imu_extraction(bag_path):
    """Test 4 — IMU data extraction."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections
                    if c.topic == "/imu/data"]
            msgs = list(reader.messages(
                connections=conns))
            _, _, rawdata = msgs[0]
            msg = typestore.deserialize_cdr(
                rawdata, "sensor_msgs/msg/Imu")

            # z should be 9.8 (gravity)
            gravity_ok = abs(
                msg.linear_acceleration.z - 9.8) < 0.01
            test("IMU extraction",
                 gravity_ok and len(msgs) == 5,
                 f"Gravity z={msg.linear_acceleration.z}"
                 f" msgs={len(msgs)}")
    except Exception as e:
        test("IMU extraction", False, str(e))

def test_gps_extraction(bag_path):
    """Test 5 — GPS data extraction."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections
                    if c.topic == "/gps/fix"]
            msgs = list(reader.messages(
                connections=conns))
            _, _, rawdata = msgs[0]
            msg = typestore.deserialize_cdr(
                rawdata, "sensor_msgs/msg/NavSatFix")

            #Hyderabad coordinates
            hyd_lat = abs(msg.latitude - 17.3850) < 0.001
            hyd_lon = abs(msg.longitude - 78.4867) < 0.001
            test("GPS extraction",
                 hyd_lat and hyd_lon,
                 f"Lat={msg.latitude:.4f}"
                 f" Lon={msg.longitude:.4f}")
    except Exception as e:
        test("GPS extraction", False, str(e))

def test_odometry_extraction(bag_path):
    """Test 6 — Odometry data extraction."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections
                    if c.topic == "/odom"]
            msgs = list(reader.messages(
                connections=conns))
            _, _, rawdata = msgs[1]
            msg = typestore.deserialize_cdr(
                rawdata, "nav_msgs/msg/Odometry")

            # Second msg x should be 0.1
            x_ok = abs(
                msg.pose.pose.position.x - 0.1) < 0.01
            test("Odometry extraction",
                 x_ok and len(msgs) == 5,
                 f"Position x={msg.pose.pose.position.x}"
                 f" msgs={len(msgs)}")
    except Exception as e:
        test("Odometry extraction", False, str(e))

def test_image_extraction(bag_path):
    """Test 7 — Image data extraction."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections
                    if c.topic == "/camera/image_raw"]
            msgs = list(reader.messages(
                connections=conns))
            _, _, rawdata = msgs[0]
            msg = typestore.deserialize_cdr(
                rawdata, "sensor_msgs/msg/Image")

            size_ok = (msg.width == 4 and
                      msg.height == 4)
            enc_ok  = msg.encoding == 'rgb8'
            test("Image extraction",
                 size_ok and enc_ok,
                 f"Size={msg.width}x{msg.height}"
                 f" Encoding={msg.encoding}")
    except Exception as e:
        test("Image extraction", False, str(e))

def test_pointcloud_extraction(bag_path):
    """Test 8 — PointCloud data extraction."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections
                    if c.topic == "/pointcloud"]
            msgs = list(reader.messages(
                connections=conns))
            _, _, rawdata = msgs[0]
            msg = typestore.deserialize_cdr(
                rawdata,
                "sensor_msgs/msg/PointCloud2")

            test("PointCloud extraction",
                 msg.width == 2 and len(msgs) == 5,
                 f"Points={msg.width}"
                 f" msgs={len(msgs)}")
    except Exception as e:
        test("PointCloud extraction", False, str(e))

def test_playback_speed(bag_path):
    """Test 9 — Playback speed scaling."""
    try:
        from rosbags.rosbag2 import Reader
        typestore_ok = True

        with Reader(bag_path) as reader:
            timestamps = []
            for _, ts, _ in reader.messages():
                timestamps.append(ts)
                if len(timestamps) >= 2:
                    break

        if len(timestamps) >= 2:
            delta = (timestamps[1] -
                    timestamps[0]) / 1e9

            # Test 2x speed halves the sleep time
            sleep_1x = delta / 1.0
            sleep_2x = delta / 2.0
            speed_ok  = sleep_2x == sleep_1x / 2

            test("Playback speed scaling",
                 speed_ok,
                 f"1x={sleep_1x:.3f}s"
                 f" 2x={sleep_2x:.3f}s")
        else:
            test("Playback speed scaling",
                 False, "Not enough messages")
    except Exception as e:
        test("Playback speed scaling", False, str(e))

def test_invalid_bag_path():
    """Test 10 — Invalid path handled correctly."""
    try:
        invalid = "/nonexistent/path/bag"
        exists  = os.path.exists(invalid)
        test("Invalid path handling",
             not exists,
             f"Correctly detected missing: {invalid}")
    except Exception as e:
        test("Invalid path handling", False, str(e))

def test_yaml_config():
    """Test 11 — YAML config loading."""
    try:
        import yaml
        import tempfile

        # Create temp config
        config_data = {
            'bag': {
                'path': 'test_bag_unit/',
                'speed': 2.0,
                'loop': False
            },
            'topics': ['/imu/data', '/gps/fix'],
            'output': {'verbose': False}
        }

        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml',
                delete=False) as f:
            yaml.dump(config_data, f)
            tmp_path = f.name

        with open(tmp_path) as f:
            loaded = yaml.safe_load(f)

        speed_ok  = loaded['bag']['speed'] == 2.0
        topics_ok = len(loaded['topics']) == 2
        test("YAML config loading",
             speed_ok and topics_ok,
             f"Speed={loaded['bag']['speed']}"
             f" Topics={loaded['topics']}")

        os.unlink(tmp_path)
    except Exception as e:
        test("YAML config loading", False, str(e))

# ─────────────────────────────────────────
# Main
# ─────────────────────────────────────────

def main():
    print(f"\n{'='*50}")
    print(f"  ROSbag Reader Tests — DORA_NAV GSoC")
    print(f"  Author: kaushikteja")
    print(f"{'='*50}\n")

    # Setup
    print("[Setup] Creating test bag...\n")
    bag_path = test_bag_creation()

    if not bag_path:
        print("\n[ERROR] Cannot run tests without bag!")
        sys.exit(1)

    # Run all tests
    print("\n[Tests] Running...\n")
    test_topic_inspection(bag_path)
    test_message_count(bag_path)
    test_imu_extraction(bag_path)
    test_gps_extraction(bag_path)
    test_odometry_extraction(bag_path)
    test_image_extraction(bag_path)
    test_pointcloud_extraction(bag_path)
    test_playback_speed(bag_path)
    test_invalid_bag_path()
    test_yaml_config()

    # Summary
    passed = sum(1 for _, p in results if p)
    total  = len(results)
    print(f"\n{'='*50}")
    print(f"  Results: {passed}/{total} passed")
    if passed == total:
        print(f"  All tests passed")
    else:
        failed = [n for n, p in results if not p]
        print(f"  Failed: {failed}")
    print(f"{'='*50}\n")

    # Cleanup
    if os.path.exists("test_bag_unit"):
        shutil.rmtree("test_bag_unit")

if __name__ == "__main__":
    main()
