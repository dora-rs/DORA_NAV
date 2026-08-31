"""
YAML Config Reader for ROSbag DORA Node
Author: kaushikteja26
Reads config.yaml and runs the rosbag reader
"""

import yaml
import sys
import os
import time
import pyarrow as pa
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

def load_config(config_path: str) -> dict:
    """Load and validate YAML config file."""

    if not os.path.exists(config_path):
        print(f"[ERROR] Config not found: {config_path}")
        sys.exit(1)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    print(f"\n{'='*50}")
    print(f"  ROSbag Config Loader — DORA_NAV GSoC #9")
    print(f"{'='*50}")
    print(f"  Config file : {config_path}")
    print(f"  Bag path    : {config['bag']['path']}")
    print(f"  Speed       : {config['bag']['speed']}x")
    print(f"  Loop        : {config['bag']['loop']}")
    print(f"  Topics      : {config['topics']}")
    print(f"{'='*50}\n")

    return config

def validate_config(config: dict) -> bool:
    """Validate config has required fields."""
    required = ['bag', 'topics']
    for field in required:
        if field not in config:
            print(f"[ERROR] Missing required field: {field}")
            return False

    bag_path = config['bag']['path']
    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag not found: {bag_path}")
        return False

    print(f"[INFO] Config validation passed ✅")
    return True

def run_from_config(config_path: str):
    """Run ROSbag reader using YAML config."""

    # Load config
    config = load_config(config_path)

    # Validate
    if not validate_config(config):
        sys.exit(1)

    # Extract settings
    bag_path   = config['bag']['path']
    speed      = config['bag'].get('speed', 1.0)
    loop       = config['bag'].get('loop', False)
    loops      = config['bag'].get('loops', 1)
    topics     = config['topics']
    verbose    = config['output'].get('verbose', True)

    print(f"[INFO] Starting playback...\n")

    total_loops = loops if loop else 1
    loop_count  = 0

    while loop_count < total_loops:
        loop_count += 1
        if loop:
            print(f"[INFO] Loop {loop_count}/{total_loops}")

        with Reader(bag_path) as reader:

            # Filter selected topics only
            connections = [
                c for c in reader.connections
                if c.topic in topics
            ]

            found_topics = [c.topic for c in connections]
            print(f"[INFO] Found topics: {found_topics}\n")

            # Warn about missing topics
            for t in topics:
                if t not in found_topics:
                    print(f"[WARN] Topic not in bag: {t}")

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

                # Deserialize
                msg = typestore.deserialize_cdr(
                    rawdata, connection.msgtype)

                if verbose:
                    print(f"  [{timestamp/1e9:.3f}s]"
                          f" {connection.topic}"
                          f" ({connection.msgtype.split('/')[-1]})")

        print(f"\n[INFO] Playback complete ✅")

    print(f"\n[INFO] All done! 🎉")


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 \
                  else "config.yaml"
    run_from_config(config_path)


if __name__ == "__main__":
    main()
