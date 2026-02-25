from pynput import keyboard
import time
from datetime import datetime
import math
import pickle
import time
import numpy as np
import json
from json import *

from dora import DoraStatus
from dora import Node
 


node = Node()

# 定义全局变量 key_value key_press_flag 和 speed_value_ratio
key_value = None
key_press_flag = 0
speed_value_ratio = None

def on_press(key):
    global key_value, key_press_flag, speed_value_ratio
    key_value = ''
    try:
        if key.char == 'w':
            key_value = 'forward'
            key_press_flag = 1
            print("Moving forward",flush = True )
        elif key.char == 'a':
            key_value = 'left'
            key_press_flag = 1
            print("Turning left",flush = True )
        elif key.char == 'd':
            key_value = 'right'
            key_press_flag = 1
            print("Turning right",flush = True )
        elif key.char == 'x':
            key_value = 'back'
            key_press_flag = 1
            print("Moving back",flush = True )
        elif key.char == '1':
            speed_value_ratio = 1
            key_press_flag = 1
            print("Speed value ratio set to 1",flush = True )
        elif key.char == '2':
            speed_value_ratio = 2
            key_press_flag = 1
            print("Speed value ratio set to 2",flush = True )
        elif key.char == '3':
            speed_value_ratio = 3
            key_press_flag = 1
            print("Speed value ratio set to 3",flush = True )
        elif key.char == 's':
            key_value = 'stop'
            key_press_flag = 1
            print("Stopping",flush = True )
        elif key.char == 'q':
            key_value = 'quit'
            key_press_flag = 1
            print("Quitting",flush = True )
            return False  # 停止监听

    except AttributeError:
        if key == keyboard.Key.up:
            key_value = 'forward'
            key_press_flag = 1
            print("Arrow key up",flush = True )
        elif key == keyboard.Key.left:
            key_value = 'left'
            key_press_flag = 1
            print("Arrow key left",flush = True )
        elif key == keyboard.Key.right:
            key_value = 'right'
            key_press_flag = 1
            print("Arrow key right",flush = True )
        elif key == keyboard.Key.down:
            key_value = 'back'
            key_press_flag = 1
            print("Arrow key down",flush = True )
            

# 打印指示信息
print("Press 'w', 'a', 'd', 'x' for forward, left, right, back")
print("Press arrow keys for up, left, right, down")
print("Press 's' to stop")
print("Press '1', '2', '3' to set speed value ratio")
print("Press 'q' to quit")

# 启动监听器
listener = keyboard.Listener(on_press=on_press)
listener.start()

# 每0.01秒刷新一次
try:
    speed_value_ratio = 1
    linear_x = 0
    linear_y = 0
    linear_z = 0
    angular_x =0
    angular_y =0
    angular_z =0
    cnt =0
    cnt_flag = 0
    while True:
        event = node.next()
        event_type = event["type"]
        if event_type == "INPUT":

            cnt = cnt +1
            if key_value == 'quit':
                break
            elif key_value == 'forward':
                linear_x = 0.2*speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
            elif key_value == 'left':
                angular_z = 0.2*speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
            elif key_value == 'right':
                angular_z = -0.2*speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
            elif key_value == 'back':
                linear_x = -0.2*speed_value_ratio
                key_press_flag = 0
                cnt_flag = 0
            elif key_value == 'stop':
                linear_x = 0
                linear_y = 0
                linear_z = 0
                angular_x =0
                angular_y =0
                angular_z =0
                key_press_flag = 0
                cnt_flag = 0
            else:
                linear_x = 0
                linear_y = 0
                linear_z = 0
                angular_x =0
                angular_y =0
                angular_z =0
                key_press_flag = 0
                cnt_flag = 0

            if key_press_flag == 0 :
                cnt_flag = cnt_flag + 1
            if cnt_flag > 5:
                key_value = ''

   
            timestamp = datetime.now().timestamp()# 获取当前时间
            #timestamp_ms = round(datetime.now().timestamp() * 1000)# 转换为毫秒格式
            print(timestamp)
      
            sentence_dict = {
                "header": {
                    "frame_id": "keyboard",
                    "seq": cnt,
                    "stamp": {
                        "sec": int(timestamp),
                        "nanosec": int((timestamp-int(timestamp))*1e9)
                    }
                },
                "linear":{
                    "x": linear_x,
                    "y": linear_y,
                    "z": linear_z
                },
                "angular":{
                    "x": angular_x,
                    "y": angular_y,
                    "z": angular_z
                }
            }
            json_string = json.dumps(sentence_dict, indent=4)  # 使用indent参数设置缩进宽度为4
            #print(json_string)
            json_bytes = json_string.encode('utf-8')
            node.send_output("CmdVelTwist",json_bytes,event["metadata"],)
    
            #print("counter: ",self.cnt1 ,"NavSatFix  pub time: ", timestamp_ms)# 打印时间戳（毫秒）
            
            key_value = ''
            #time.sleep(0.02)
        elif event_type == "STOP":
            print("received stop")
            break
        else:
            print("received unexpected event:", event_type)
            break    
        
except KeyboardInterrupt:
    print("\nProgram interrupted by user")

# 打印最后按键的值和速度比率
print("Last key pressed:", key_value)
print("Speed value ratio:", speed_value_ratio)