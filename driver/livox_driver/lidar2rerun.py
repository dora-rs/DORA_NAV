import dora
import rerun as rr
import numpy as np
import pyarrow as pa

# ==========================================
# 全局配置参数 / Global Configurations
# ==========================================
ENABLE_DEBUG = False          # True: 开启调试打印 | False: 关闭调试打印
POINT_COLOR = [255, 0, 0]    # RGB 颜色设置, [255, 0, 0] 为纯红色

def debug_print(message):
    """辅助函数：根据全局变量决定是否打印调试信息"""
    if ENABLE_DEBUG:
        print(message)

def main():
    # 1. 初始化 Rerun
    rr.init("dora_pointcloud_visualizer", spawn=True)

    # 2. 初始化 Dora 节点
    node = dora.Node()
    print("[INFO] Rerun visualizer node started. Listening for 'pointcloud_rs'...")

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pointcloud_rs":
                raw_data = event["value"]
                
                debug_print(f"\n[DEBUG] Received INPUT event, ID: '{event['id']}'")
                debug_print(f"[DEBUG] -> Raw data type: {type(raw_data)}")
                
                if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                    data_bytes = raw_data.to_numpy(zero_copy_only=False).tobytes()
                else:
                    data_bytes = bytes(raw_data) 

                data_len = len(data_bytes)
                debug_print(f"[DEBUG] -> Total bytes parsed: {data_len}")

                if data_len < 16:
                    print("[ERROR] -> Data length is less than 16 bytes. Skipping this frame.")
                    continue

                points_payload_len = data_len - 16
                expected_points = points_payload_len // 16
                debug_print(f"[DEBUG] -> Expected number of points to parse: {expected_points}")

                if expected_points == 0:
                    debug_print("[WARNING] -> Number of points is 0! (Header only, no data)")
                    continue

                # ---- 数据解析 ----
                try:
                    # 解析 Header (seq 和 timestamp)
                    seq = np.frombuffer(data_bytes, dtype=np.uint32, count=1, offset=0)[0]
                    timestamp_us = np.frombuffer(data_bytes, dtype=np.uint64, count=1, offset=8)[0]

                    # 解析点云坐标 (只取前 3 个 float 作为 XYZ，忽略 intensity)
                    points_raw = np.frombuffer(data_bytes, dtype=np.float32, offset=16)
                    points_reshaped = points_raw.reshape((-1, 4))
                    positions = points_reshaped[:, :3]

                    # ---- Rerun 可视化 ----
                    rr.set_time_sequence("sequence_id", int(seq))
                    rr.set_time_nanos("timestamp", int(timestamp_us * 1000))

                    # 传入全局变量 POINT_COLOR，Rerun 会自动将其应用到所有点
                    rr.log(
                        "world/pointcloud", 
                        rr.Points3D(positions=positions, colors=POINT_COLOR, radii=0.02)
                    )
                    debug_print(f"[DEBUG] -> Successfully sent {expected_points} points to Rerun!")
                    
                except Exception as e:
                    print(f"[ERROR] -> Exception occurred during parsing or sending to Rerun: {e}\n")

        elif event["type"] == "STOP":
            print("[INFO] Received STOP event. Exiting...")
            break

        elif event["type"] == "ERROR":
            print(f"[ERROR] Received ERROR event: {event['error']}")

if __name__ == "__main__":
    main()