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

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 rosbag_reader.py <bag_path> [--imu] [--gps]")
        sys.exit(1)

    bag_path = sys.argv[1]
    args = sys.argv[2:]

    inspect_rosbag(bag_path)

    if "--imu" in args:
        extract_imu(bag_path)

    if "--gps" in args:
        extract_gps(bag_path)

    if not args:
        extract_imu(bag_path)
        extract_gps(bag_path)

if __name__ == "__main__":
    main()
