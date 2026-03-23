"""
ROSbag Reader for DORA_NAV — GSoC 2026 Project #9
Author: kaushikteja26
Features:
- Inspect bag topics
- Extract IMU data
- Extract GPS data
- Timed playback preview
"""

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import sys
import os

typestore = get_typestore(Stores.ROS2_HUMBLE)

def inspect_rosbag(bag_path: str):
    """Inspect a ROSbag file and print available topics."""
    if not os.path.exists(bag_path):
        print(f"Error: Bag file not found at {bag_path}")
        return

    print(f"\n{'='*50}")
    print(f"  ROSbag Inspector — DORA_NAV GSoC #9")
    print(f"{'='*50}")
    print(f"  Bag: {bag_path}\n")

    with Reader(bag_path) as reader:
        print(f"  Duration:      {reader.duration / 1e9:.2f} seconds")
        print(f"  Message count: {reader.message_count}")
        print(f"\n  Available Topics:")
        print(f"  {'-'*44}")

        for connection in reader.connections:
            print(f"  Topic:  {connection.topic}")
            print(f"  Type:   {connection.msgtype}")
            print(f"  Count:  {connection.msgcount}")
            print()

def extract_imu(bag_path: str, topic: str = "/imu/data", max_msgs: int = 5):
    """Extract and display IMU messages."""
    print(f"\n{'='*50}")
    print(f"  IMU Data — topic: {topic}")
    print(f"{'='*50}")

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            print(f"  No IMU topic found: {topic}")
            return

        count = 0
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if count >= max_msgs:
                break
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print(f"  Time: {timestamp / 1e9:.3f}s")
            print(f"  Linear Accel — x:{msg.linear_acceleration.x:.3f}"
                  f" y:{msg.linear_acceleration.y:.3f}"
                  f" z:{msg.linear_acceleration.z:.3f}")
            print(f"  Angular Vel  — x:{msg.angular_velocity.x:.3f}"
                  f" y:{msg.angular_velocity.y:.3f}"
                  f" z:{msg.angular_velocity.z:.3f}")
            print()
            count += 1

def extract_gps(bag_path: str, topic: str = "/gps/fix", max_msgs: int = 5):
    """Extract and display GPS messages."""
    print(f"\n{'='*50}")
    print(f"  GPS Data — topic: {topic}")
    print(f"{'='*50}")

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            print(f"  No GPS topic found: {topic}")
            return

        count = 0
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if count >= max_msgs:
                break
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print(f"  Time:      {timestamp / 1e9:.3f}s")
            print(f"  Latitude:  {msg.latitude:.6f}")
            print(f"  Longitude: {msg.longitude:.6f}")
            print(f"  Altitude:  {msg.altitude:.2f}m")
            print()
            count += 1
def extract_odometry(bag_path: str, topic: str = "/odom", max_msgs: int = 5):
    """Extract and display Odometry messages."""
    print(f"\n{'='*50}")
    print(f"  Odometry Data — topic: {topic}")
    print(f"{'='*50}")

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections 
                      if c.topic == topic]
        if not connections:
            print(f"  No Odometry topic found: {topic}")
            return

        count = 0
        for connection, timestamp, rawdata in \
                reader.messages(connections=connections):
            if count >= max_msgs:
                break
            msg = typestore.deserialize_cdr(
                rawdata, connection.msgtype)
            print(f"  Time:     {timestamp / 1e9:.3f}s")
            print(f"  Position  x:{msg.pose.pose.position.x:.3f}"
                  f" y:{msg.pose.pose.position.y:.3f}"
                  f" z:{msg.pose.pose.position.z:.3f}")
            print(f"  Velocity  x:{msg.twist.twist.linear.x:.3f}"
                  f" y:{msg.twist.twist.linear.y:.3f}")
            print()
            count += 1

def extract_image(bag_path: str, 
                  topic: str = "/camera/image_raw", 
                  max_msgs: int = 3):
    """Extract and display Image messages."""
    print(f"\n{'='*50}")
    print(f"  Image Data — topic: {topic}")
    print(f"{'='*50}")

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections 
                      if c.topic == topic]
        if not connections:
            print(f"  No Image topic found: {topic}")
            return

        count = 0
        for connection, timestamp, rawdata in \
                reader.messages(connections=connections):
            if count >= max_msgs:
                break
            msg = typestore.deserialize_cdr(
                rawdata, connection.msgtype)
            print(f"  Time:      {timestamp / 1e9:.3f}s")
            print(f"  Width:     {msg.width}px")
            print(f"  Height:    {msg.height}px")
            print(f"  Encoding:  {msg.encoding}")
            print(f"  Data size: {len(msg.data)} bytes")
            print()
            count += 1

def extract_pointcloud(bag_path: str,
                       topic: str = "/pointcloud",
                       max_msgs: int = 3):
    """Extract and display PointCloud2 messages."""
    print(f"\n{'='*50}")
    print(f"  PointCloud2 Data — topic: {topic}")
    print(f"{'='*50}")

    import numpy as np

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections 
                      if c.topic == topic]
        if not connections:
            print(f"  No PointCloud topic found: {topic}")
            return

        count = 0
        for connection, timestamp, rawdata in \
                reader.messages(connections=connections):
            if count >= max_msgs:
                break
            msg = typestore.deserialize_cdr(
                rawdata, connection.msgtype)

            # Parse XYZ points from raw bytes
            points = np.frombuffer(
                bytes(msg.data), dtype=np.float32
            ).reshape(-1, 3)

            print(f"  Time:        {timestamp / 1e9:.3f}s")
            print(f"  Num points:  {msg.width}")
            print(f"  Point step:  {msg.point_step} bytes")
            print(f"  First 3 points (x, y, z):")
            for p in points[:3]:
                print(f"    x:{p[0]:.2f} y:{p[1]:.2f} z:{p[2]:.2f}")
            print()
            count += 1
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 rosbag_reader.py <bag_path> "
              "[--imu] [--gps] [--odom] [--image] [--pointcloud]")
        sys.exit(1)

    bag_path = sys.argv[1]
    args = sys.argv[2:]

    # Always inspect first
    inspect_rosbag(bag_path)

    # Extract specific or all
    if "--imu" in args:
        extract_imu(bag_path)
    if "--gps" in args:
        extract_gps(bag_path)
    if "--odom" in args:
        extract_odometry(bag_path)
    if "--image" in args:
        extract_image(bag_path)
    if "--pointcloud" in args:
        extract_pointcloud(bag_path)

    # Auto extract all if no flags
    if not args:
        extract_imu(bag_path)
        extract_gps(bag_path)
        extract_odometry(bag_path)
        extract_image(bag_path)
        extract_pointcloud(bag_path)

if __name__ == "__main__":
    main()
