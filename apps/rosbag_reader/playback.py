"""
ROSbag Timed Playback for DORA_NAV — GSoC 2026 Project #9
Author: kaushikteja26
Features:
- Real-time playback following original timestamps
- Speed scaling (0.5x, 1x, 2x etc)
- Loop playback
- Start/stop control
"""

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import time
import sys
import os

typestore = get_typestore(Stores.ROS2_HUMBLE)

def play_rosbag(bag_path: str, speed: float = 1.0, 
                loop: bool = False, max_loops: int = 1):
    """
    Play a ROSbag file with timed playback.
    
    Args:
        bag_path: Path to the bag folder
        speed: Playback speed (0.5 = half speed, 2.0 = double speed)
        loop: Whether to loop playback
        max_loops: How many times to loop (if loop=True)
    """

    if not os.path.exists(bag_path):
        print(f"Error: Bag not found at {bag_path}")
        return

    print(f"\n{'='*50}")
    print(f"  ROSbag Player — DORA_NAV GSoC ")
    print(f"{'='*50}")
    print(f"  Bag:   {bag_path}")
    print(f"  Speed: {speed}x")
    print(f"  Loop:  {loop}")
    print(f"{'='*50}\n")

    loop_count = 0
    total_loops = max_loops if loop else 1

    while loop_count < total_loops:
        loop_count += 1
        if loop:
            print(f"   Loop {loop_count}/{total_loops}\n")

        with Reader(bag_path) as reader:
            prev_timestamp = None
            msg_count = 0

            for connection, timestamp, rawdata in reader.messages():
                # Calculate delay based on timestamps
                if prev_timestamp is not None:
                    # Time difference in seconds
                    delta = (timestamp - prev_timestamp) / 1e9
                    # Apply speed scaling
                    sleep_time = delta / speed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                prev_timestamp = timestamp
                msg_count += 1

                # Deserialize and display message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                display_message(connection.topic, connection.msgtype, 
                              timestamp, msg)

        print(f"\n  Playback complete — {msg_count} messages played")

    print(f"\n   All done!")


def display_message(topic: str, msgtype: str, 
                   timestamp: int, msg):
    """Display a message based on its type."""
    
    t = timestamp / 1e9

    if "Imu" in msgtype:
        print(f"  [{t:.3f}s] IMU | {topic}")
        print(f"    Accel x:{msg.linear_acceleration.x:.3f}"
              f" y:{msg.linear_acceleration.y:.3f}"
              f" z:{msg.linear_acceleration.z:.3f}")
        print(f"    Gyro  x:{msg.angular_velocity.x:.3f}"
              f" y:{msg.angular_velocity.y:.3f}"
              f" z:{msg.angular_velocity.z:.3f}")

    elif "NavSatFix" in msgtype:
        print(f"  [{t:.3f}s] GPS | {topic}")
        print(f"    Lat:{msg.latitude:.6f}"
              f" Lon:{msg.longitude:.6f}"
              f" Alt:{msg.altitude:.1f}m")

    else:
        print(f"  [{t:.3f}s] {topic} ({msgtype})")

    print()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 playback.py <bag_path> "
              "[--speed 1.0] [--loop] [--loops 3]")
        print("\nExamples:")
        print("  python3 playback.py test_bag/")
        print("  python3 playback.py test_bag/ --speed 2.0")
        print("  python3 playback.py test_bag/ --speed 0.5")
        print("  python3 playback.py test_bag/ --loop --loops 3")
        sys.exit(1)

    bag_path = sys.argv[1]
    args = sys.argv[2:]

    # Parse arguments
    speed = 1.0
    loop = False
    max_loops = 2

    if "--speed" in args:
        idx = args.index("--speed")
        speed = float(args[idx + 1])

    if "--loop" in args:
        loop = True

    if "--loops" in args:
        idx = args.index("--loops")
        max_loops = int(args[idx + 1])

    play_rosbag(bag_path, speed=speed, 
                loop=loop, max_loops=max_loops)


if __name__ == "__main__":
    main()
