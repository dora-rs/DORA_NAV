driver文件主要存放dora支持的传感器驱动节点
目前支持的传感器如下
 - camera: usb camera (OpenCV)
 - lidar：Mid360   robsense airy   RSHELIOS  RSHELIOS_16P
 - IMU: HWT9053 、SANCHI
 - GNSS: ublox for NEMA-0183 protocol



 ## 1. rslidar_driver 
使用是需要注意修改CMakeLists.txt文件中，第45-46行、第58行引用的DORA的目录

在rslidar_driver目录下执行编译指令

```
cd rslidar_driver
mkdir build && cd build
cmake ..
make
```

### 启动节点
雷达传感器读取数据的指令:
```
dora up
dora run driver_rslidar.yml
```

参数说明
LIDAR_TYPE: 激光雷达的类型 RSHELIOS_16P  # RSAIRY RSHELIOS RSHELIOS_16P
ONLINE_LIDAR: 表示使用在线激光雷达函数离线激光雷达  0  # i: online   0: offine
PCAP_PATH:  pcap文件存放的路径

### 测试数据：

- fast_stable_straight.pcap 是利用 RSHELIOS_16P 激光雷达录制的

- lidar.pcap 、lidar2.pcap、lidar3.pcap 是利用 RSHELIOS 激光雷达录制的

数据集可以从百度网盘下载 
链接: https://pan.baidu.com/s/1i9qaw3oh3N0OUbmQJRP0cQ?pwd=f8mx 提取码: f8mx 

## 2. Mid360 Lidar driver 

第1次运行的时候需要下载安装 Livox-SDK2
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

编译
```
cd livox_driver
mkdir build
cd build
cmake ..
make
cd ..
```
启动livox_driver节点
```
dora up
dora run livox_dora.yml
```
## 3. pointcloud_merger 
激光点云拼接节点，接收输入2个激光雷达的话题，根据multi_lidar_calibration节点离线标定的旋转和平移参数实现点云拼接
编译
```
cd pointcloud_merger
mkdir build
cd build
cmake ..
make
cd ..
```
启动livox_driver节点
```
dora up
dora run pointcloud_merger.yml
```
## 4. multi_lidar_calibration 

激光雷达标定节点，输入2个激光雷达的话题，计算两个激光雷达之间的旋转和平移

