"""
Creates a synthetic ROSbag v2 with ALL sensor types
for testing the DORA ROSbag reader
"""
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
import numpy as np
import pathlib
import shutil

bag_path = pathlib.Path("test_bag")

# Remove if exists
if bag_path.exists():
    shutil.rmtree(bag_path)

typestore = get_typestore(Stores.ROS2_HUMBLE)

with Writer(bag_path, version=9) as writer:

    # ── IMU ──────────────────────────────────
    imu_conn = writer.add_connection(
        "/imu/data", "sensor_msgs/msg/Imu",
        typestore=typestore)

    # ── GPS ──────────────────────────────────
    gps_conn = writer.add_connection(
        "/gps/fix", "sensor_msgs/msg/NavSatFix",
        typestore=typestore)

    # ── Odometry ─────────────────────────────
    odom_conn = writer.add_connection(
        "/odom", "nav_msgs/msg/Odometry",
        typestore=typestore)

    # ── Image ─────────────────────────────────
    img_conn = writer.add_connection(
        "/camera/image_raw", "sensor_msgs/msg/Image",
        typestore=typestore)

    # ── PointCloud2 ───────────────────────────
    pc_conn = writer.add_connection(
        "/pointcloud", "sensor_msgs/msg/PointCloud2",
        typestore=typestore)

    for i in range(10):
        timestamp = i * 100_000_000  # 100ms apart

        header = typestore.types['std_msgs/msg/Header'](
            stamp=typestore.types['builtin_interfaces/msg/Time'](
                sec=i, nanosec=0),
            frame_id='base_link')

        # ── Write IMU ────────────────────────
        imu_msg = typestore.types['sensor_msgs/msg/Imu'](
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
        writer.write(imu_conn, timestamp,
                     typestore.serialize_cdr(
                         imu_msg, 'sensor_msgs/msg/Imu'))

        # ── Write GPS ────────────────────────
        gps_msg = typestore.types['sensor_msgs/msg/NavSatFix'](
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
        writer.write(gps_conn, timestamp,
                     typestore.serialize_cdr(
                         gps_msg, 'sensor_msgs/msg/NavSatFix'))

        # ── Write Odometry ───────────────────
        odom_msg = typestore.types['nav_msgs/msg/Odometry'](
            header=header,
            child_frame_id='base_link',
            pose=typestore.types[
                'geometry_msgs/msg/PoseWithCovariance'](
                pose=typestore.types['geometry_msgs/msg/Pose'](
                    position=typestore.types[
                        'geometry_msgs/msg/Point'](
                        x=0.1*i, y=0.05*i, z=0.0),
                    orientation=typestore.types[
                        'geometry_msgs/msg/Quaternion'](
                        x=0.0, y=0.0, z=0.0, w=1.0)),
                covariance=np.zeros(36)),
            twist=typestore.types[
                'geometry_msgs/msg/TwistWithCovariance'](
                twist=typestore.types['geometry_msgs/msg/Twist'](
                    linear=typestore.types[
                        'geometry_msgs/msg/Vector3'](
                        x=0.1, y=0.0, z=0.0),
                    angular=typestore.types[
                        'geometry_msgs/msg/Vector3'](
                        x=0.0, y=0.0, z=0.01)),
                covariance=np.zeros(36)),
        )
        writer.write(odom_conn, timestamp,
                     typestore.serialize_cdr(
                         odom_msg, 'nav_msgs/msg/Odometry'))

        # ── Write Image ──────────────────────
        # Simple 4x4 RGB image
        img_data = np.random.randint(
            0, 255, (4, 4, 3), dtype=np.uint8)
        img_msg = typestore.types['sensor_msgs/msg/Image'](
            header=header,
            height=4,
            width=4,
            encoding='rgb8',
            is_bigendian=False,
            step=12,  # width * 3 channels
            data=img_data.flatten(),
        )
        writer.write(img_conn, timestamp,
                     typestore.serialize_cdr(
                         img_msg, 'sensor_msgs/msg/Image'))

        # ── Write PointCloud2 ────────────────
        # Simple 5 point cloud
        points = np.array([
            [1.0*i, 0.0, 0.5],
            [2.0*i, 0.5, 1.0],
            [3.0*i, 1.0, 1.5],
            [4.0*i, 1.5, 2.0],
            [5.0*i, 2.0, 2.5],
        ], dtype=np.float32)

        pc_data = points.tobytes()

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

        pc_msg = typestore.types[
            'sensor_msgs/msg/PointCloud2'](
            header=header,
            height=1,
            width=5,
            fields=fields,
            is_bigendian=False,
            point_step=12,  # 3 floats * 4 bytes
            row_step=60,    # 5 points * 12 bytes
            data=np.frombuffer(pc_data, dtype=np.uint8),
            is_dense=True,
        )
        writer.write(pc_conn, timestamp,
                     typestore.serialize_cdr(
                         pc_msg,
                         'sensor_msgs/msg/PointCloud2'))

print("Test bag created with ALL sensor types")
print("Topics: IMU, GPS, Odometry, Image, PointCloud2")
