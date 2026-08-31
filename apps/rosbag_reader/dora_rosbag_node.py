"""
ROSbag Reader DORA Node — GSoC 2026 Project #9
Author: kaushikteja26

This is the actual DORA node that:
1. Reads a ROSbag file
2. Parses sensor messages
3. Publishes them into DORA dataflow pipeline
"""

import dora
import pyarrow as pa
import time
import os
import sys
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

# ─────────────────────────────────────────
# Converters: ROS message → Apache Arrow
# ─────────────────────────────────────────

def imu_to_arrow(msg) -> pa.StructArray:
    """Convert ROS IMU message to Apache Arrow."""
    return pa.array([{
        "linear_acceleration_x": float(msg.linear_acceleration.x),
        "linear_acceleration_y": float(msg.linear_acceleration.y),
        "linear_acceleration_z": float(msg.linear_acceleration.z),
        "angular_velocity_x":    float(msg.angular_velocity.x),
        "angular_velocity_y":    float(msg.angular_velocity.y),
        "angular_velocity_z":    float(msg.angular_velocity.z),
    }], type=pa.struct([
        ("linear_acceleration_x", pa.float64()),
        ("linear_acceleration_y", pa.float64()),
        ("linear_acceleration_z", pa.float64()),
        ("angular_velocity_x",    pa.float64()),
        ("angular_velocity_y",    pa.float64()),
        ("angular_velocity_z",    pa.float64()),
    ]))

def gps_to_arrow(msg) -> pa.StructArray:
    """Convert ROS NavSatFix message to Apache Arrow."""
    return pa.array([{
        "latitude":  float(msg.latitude),
        "longitude": float(msg.longitude),
        "altitude":  float(msg.altitude),
    }], type=pa.struct([
        ("latitude",  pa.float64()),
        ("longitude", pa.float64()),
        ("altitude",  pa.float64()),
    ]))

def odometry_to_arrow(msg) -> pa.StructArray:
    """Convert ROS Odometry message to Apache Arrow."""
    return pa.array([{
        "pos_x": float(msg.pose.pose.position.x),
        "pos_y": float(msg.pose.pose.position.y),
        "pos_z": float(msg.pose.pose.position.z),
        "vel_x": float(msg.twist.twist.linear.x),
        "vel_y": float(msg.twist.twist.linear.y),
    }], type=pa.struct([
        ("pos_x", pa.float64()),
        ("pos_y", pa.float64()),
        ("pos_z", pa.float64()),
        ("vel_x", pa.float64()),
        ("vel_y", pa.float64()),
    ]))

# ─────────────────────────────────────────
# Topic → Output name + converter mapping
# ─────────────────────────────────────────

TOPIC_MAP = {
    "/imu/data":  ("imu_data",  imu_to_arrow),
    "/gps/fix":   ("gps_data",  gps_to_arrow),
    "/odom":      ("odom_data", odometry_to_arrow),
}

# ─────────────────────────────────────────
# Main DORA Node
# ─────────────────────────────────────────

def run_dora_node(bag_path: str, speed: float = 1.0, 
                  loop: bool = False):
    """
    Main DORA node that reads ROSbag and 
    publishes data into DORA dataflow.
    """

    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag not found: {bag_path}")
        sys.exit(1)

    print(f"\n[DORA ROSbag Node] Starting...")
    print(f"  Bag:   {bag_path}")
    print(f"  Speed: {speed}x")
    print(f"  Loop:  {loop}\n")

    # Initialize DORA node
    node = dora.Node()
    print("[DORA ROSbag Node] Connected to DORA dataflow \n")

    keep_running = True

    while keep_running:
        with Reader(bag_path) as reader:

            # Filter only topics we support
            connections = [
                c for c in reader.connections
                if c.topic in TOPIC_MAP
            ]

            if not connections:
                print("[WARN] No supported topics found in bag!")
                print(f"  Supported: {list(TOPIC_MAP.keys())}")
                break

            print(f"[INFO] Publishing topics:")
            for c in connections:
                out_name, _ = TOPIC_MAP[c.topic]
                print(f"  {c.topic} → DORA output: '{out_name}'")
            print()

            prev_timestamp = None

            for connection, timestamp, rawdata in \
                    reader.messages(connections=connections):

                # Timed playback
                if prev_timestamp is not None:
                    delta = (timestamp - prev_timestamp) / 1e9
                    sleep_time = delta / speed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                prev_timestamp = timestamp

                # Get output name and converter
                out_name, converter = TOPIC_MAP[connection.topic]

                # Deserialize ROS message
                msg = typestore.deserialize_cdr(
                    rawdata, connection.msgtype
                )

                # Convert to Arrow
                arrow_data = converter(msg)

                # Publish to DORA
                node.send_output(out_name, arrow_data)

                print(f"  [{timestamp/1e9:.3f}s] "
                      f"Published → {out_name}")

        if not loop:
            keep_running = False
        else:
            print("\n[INFO] Looping playback...\n")

    print("\n[DORA ROSbag Node] Done")


def main():
    # Get config from environment variables
    # (DORA nodes get config this way)
    bag_path = os.environ.get("BAG_PATH", "test_bag/")
    speed    = float(os.environ.get("SPEED", "1.0"))
    loop     = os.environ.get("LOOP", "false").lower() == "true"

    run_dora_node(bag_path, speed=speed, loop=loop)


if __name__ == "__main__":
    main()
