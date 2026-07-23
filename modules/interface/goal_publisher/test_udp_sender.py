#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP Goal Sender - 用于测试goal_publisher节点的UDP接收功能
发送格式: {"x": <double>, "y": <double>, "yaw": <double>}
"""

import socket
import json
import time
import sys


def send_goal(sock, host, port, x, y, yaw):
    """发送单个目标点"""
    goal = {
        "x": x,
        "y": y,
        "yaw": yaw
    }

    # 转换为JSON字符串并编码
    message = json.dumps(goal).encode('utf-8')

    # 发送UDP数据包
    sock.sendto(message, (host, port))

    print(f"✓ 已发送目标点: x={x}, y={y}, yaw={yaw}")
    print(f"  JSON: {json.dumps(goal)}")
    return True


def main():
    # 配置
    HOST = 'localhost'  # 目标主机（如果goal_publisher在远程，修改为对应IP）
    PORT = 9999         # UDP端口

    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("=" * 60)
    print("UDP Goal Sender - 目标点发送测试工具")
    print("=" * 60)
    print(f"目标地址: {HOST}:{PORT}")
    print()

    # 测试用例1: 发送单个目标点
    print("【测试1】发送单个目标点")
    send_goal(sock, HOST, PORT, 5.0, 3.0, 1.57)
    print()

    time.sleep(1)

    # # 测试用例2: 发送一系列目标点
    # print("【测试2】发送一系列目标点")
    # test_goals = [
    #     (10.0, 5.0, 0.0),
    #     (15.0, 10.0, 1.57),
    #     (20.0, 15.0, 3.14),
    #     (0.0, 0.0, 0.0)
    # ]

    # for i, (x, y, yaw) in enumerate(test_goals, 1):
    #     print(f"目标点 {i}/{len(test_goals)}:")
    #     send_goal(sock, HOST, PORT, x, y, yaw)
    #     time.sleep(2)  # 每个目标点间隔2秒
    #     print()

    # # 测试用例3: 不带yaw字段（测试可选字段）
    # print("【测试3】发送不带yaw字段的目标点")
    # goal_no_yaw = {"x": 8.0, "y": 6.0}
    # message = json.dumps(goal_no_yaw).encode('utf-8')
    # sock.sendto(message, (HOST, PORT))
    # print(f"✓ 已发送: {json.dumps(goal_no_yaw)}")
    # print()

    # time.sleep(1)

    # # 测试用例4: 发送错误格式（测试错误处理）
    # print("【测试4】发送错误格式（测试错误处理）")

    # # 缺少必需字段
    # invalid_goal1 = {"x": 5.0}  # 缺少y字段
    # message = json.dumps(invalid_goal1).encode('utf-8')
    # sock.sendto(message, (HOST, PORT))
    # print(f"✗ 已发送无效数据（缺少y字段）: {json.dumps(invalid_goal1)}")

    # time.sleep(0.5)

    # # 错误的数据类型
    # invalid_goal2 = {"x": "five", "y": 3.0}  # x是字符串而非数字
    # message = json.dumps(invalid_goal2).encode('utf-8')
    # sock.sendto(message, (HOST, PORT))
    # print(f"✗ 已发送无效数据（错误类型）: {json.dumps(invalid_goal2)}")

    # time.sleep(0.5)

    # # 无效的JSON
    # invalid_json = b"{x:5.0,y:3.0}"  # 缺少引号
    # sock.sendto(invalid_json, (HOST, PORT))
    # print(f"✗ 已发送无效JSON: {invalid_json.decode('utf-8')}")
    # print()

    # 关闭socket
    sock.close()

    print("=" * 60)
    print("测试完成！")
    print("请检查goal_publisher节点的输出日志")
    print("=" * 60)


def interactive_mode():
    """交互模式：手动输入目标点"""
    HOST = 'localhost'
    PORT = 9999

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("=" * 60)
    print("交互模式 - 手动发送目标点")
    print("=" * 60)
    print(f"目标地址: {HOST}:{PORT}")
    print("输入 'q' 或 'quit' 退出")
    print()

    while True:
        try:
            # 输入坐标
            x_input = input("请输入 x 坐标 (或 q 退出): ").strip()
            if x_input.lower() in ['q', 'quit']:
                break

            y_input = input("请输入 y 坐标: ").strip()
            yaw_input = input("请输入 yaw 角度 (可选，直接回车跳过): ").strip()

            # 转换为数字
            x = float(x_input)
            y = float(y_input)
            yaw = float(yaw_input) if yaw_input else 0.0

            # 发送
            send_goal(sock, HOST, PORT, x, y, yaw)
            print()

        except ValueError:
            print("✗ 输入格式错误，请输入有效的数字")
            print()
        except KeyboardInterrupt:
            print("\n\n中断退出")
            break

    sock.close()
    print("已退出")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "-i":
        # 交互模式
        interactive_mode()
    else:
        # 自动测试模式
        main()
