"""
Navigation Stub Node — simulates navigation stack
Replace with real nav in production
Author: kaushikteja26
"""
import dora
import pyarrow as pa

def main():
    node = dora.Node()
    print("\n[Nav Node] Started — waiting for GPS + Odom...\n")
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "gps":
                print(f"  [Nav] Got GPS data")
                position = pa.array([{
                    "x": 10.0, "y": 5.0,
                }], type=pa.struct([
                    ("x", pa.float64()),
                    ("y", pa.float64()),
                ]))
                node.send_output("current_position", position)
                print(f"  [Nav] Published position ✅")
            elif event["id"] == "odom":
                print(f"  [Nav] Got Odometry — updating path")
                path = pa.array([{
                    "waypoint_x": 20.0,
                    "waypoint_y": 10.0,
                }], type=pa.struct([
                    ("waypoint_x", pa.float64()),
                    ("waypoint_y", pa.float64()),
                ]))
                node.send_output("planned_path", path)
                print(f"  [Nav] Published path ✅")

if __name__ == "__main__":
    main()
