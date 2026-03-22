"""
Proof of Concept: ROSbag Reader for DORA_NAV
Author: kaushikteja
Description: Reads a ROSbag v2 file and prints available topics
This is a preview of GSoC Project #9 implementation
"""

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import sys
import os

def inspect_rosbag(bag_path: str):
    """Inspect a ROSbag file and print available topics and message types."""
    
    if not os.path.exists(bag_path):
        print(f"Error: Bag file not found at {bag_path}")
        return
    
    print(f"\n{'='*50}")
    print(f"ROSbag Inspector — DORA_NAV POC")
    print(f"{'='*50}")
    print(f"Bag path: {bag_path}\n")
    
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with Reader(bag_path) as reader:
        # Print bag info
        print(f"Duration:     {reader.duration / 1e9:.2f} seconds")
        print(f"Message count:{reader.message_count}")
        print(f"\nAvailable Topics:")
        print(f"{'-'*50}")
        
        for connection in reader.connections:
            print(f"  Topic:    {connection.topic}")
            print(f"  Type:     {connection.msgtype}")
            print(f"  Count:    {connection.msgcount}")
            print()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 rosbag_reader.py <path_to_bag>")
        print("Example: python3 rosbag_reader.py my_recording/")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    inspect_rosbag(bag_path)

if __name__ == "__main__":
    main()
