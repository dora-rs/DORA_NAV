"""
Creates a synthetic ROSbag v2 file for testing the DORA reader
"""
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros2_humble import (
    sensor_msgs__msg__Imu as Imu,
    sensor_msgs__msg__NavSatFix as NavSatFix,
    std_msgs__msg__String as String,
)
import numpy as np
import pathlib
import shutil

bag_path = pathlib.Path("test_bag")

# Remove if exists
if bag_path.exists():
    shutil.rmtree(bag_path)

typestore = get_typestore(Stores.ROS2_HUMBLE)

with Writer(bag_path, version=9) as writer:
    # Add IMU topic
    imu_conn = writer.add_connection(
        "/imu/data",
        "sensor_msgs/msg/Imu",
        typestore=typestore
    )

    # Add GPS topic
    gps_conn = writer.add_connection(
        "/gps/fix",
        "sensor_msgs/msg/NavSatFix",
        typestore=typestore
    )

    # Write 10 IMU messages
    for i in range(10):
        timestamp = i * 100_000_000  # 100ms apart
        imu_msg = Imu(
            header=typestore.types['std_msgs/msg/Header'](
                stamp=typestore.types['builtin_interfaces/msg/Time'](
                    sec=i, nanosec=0
                ),
                frame_id='imu_link'
            ),
            linear_acceleration=typestore.types[
                'geometry_msgs/msg/Vector3'](
                x=0.1*i, y=0.0, z=9.8
            ),
            angular_velocity=typestore.types[
                'geometry_msgs/msg/Vector3'](
                x=0.0, y=0.0, z=0.01*i
            ),
            orientation=typestore.types[
                'geometry_msgs/msg/Quaternion'](
                x=0.0, y=0.0, z=0.0, w=1.0
            ),
            linear_acceleration_covariance=np.zeros(9),
            angular_velocity_covariance=np.zeros(9),
            orientation_covariance=np.zeros(9),
        )
        writer.write(imu_conn, timestamp,
                     typestore.serialize_cdr(imu_msg,
                     'sensor_msgs/msg/Imu'))

    # Write 10 GPS messages
    for i in range(10):
        timestamp = i * 100_000_000
        gps_msg = NavSatFix(
            header=typestore.types['std_msgs/msg/Header'](
                stamp=typestore.types['builtin_interfaces/msg/Time'](
                    sec=i, nanosec=0
                ),
                frame_id='gps_link'
            ),
            latitude=37.7749 + i*0.0001,
            longitude=-122.4194 + i*0.0001,
            altitude=10.0,
            status=typestore.types[
                'sensor_msgs/msg/NavSatStatus'](
                status=0, service=1
            ),
            position_covariance=np.zeros(9),
            position_covariance_type=0,
        )
        writer.write(gps_conn, timestamp,
                     typestore.serialize_cdr(gps_msg,
                     'sensor_msgs/msg/NavSatFix'))

print("✅ Test bag created at: ./test_bag/")
