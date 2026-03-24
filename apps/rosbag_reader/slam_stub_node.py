"""
SLAM Stub Node — simulates SLAM algorithm
Replace with real SLAM in production
Author: kaushikteja26
"""
import dora
import pyarrow as pa

def main():
    node = dora.Node()
    print("\n[SLAM Node] Started — waiting for LiDAR + IMU...\n")
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pointcloud":
                print(f"  [SLAM] Processing PointCloud...")
                pose = pa.array([{
                    "x": 1.0, "y": 0.5, "theta": 0.1
                }], type=pa.struct([
                    ("x", pa.float64()),
                    ("y", pa.float64()),
                    ("theta", pa.float64()),
                ]))
                node.send_output("slam_pose", pose)
                print(f"  [SLAM] Published pose ✅")
            elif event["id"] == "imu":
                print(f"  [SLAM] Got IMU for sensor fusion")

if __name__ == "__main__":
    main()
