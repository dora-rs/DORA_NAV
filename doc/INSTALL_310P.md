## 昇腾310P OpenEuler系统
### 前置环境安装
```shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh  #安装rust工具链
sudo dnf install gcc gcc-c++ make -y  #安装C/C++编译工具链
sudo dnf install cmake -y #安装cmake
sudo dnf install pcl-devel pcl-tools -y #安装PCL库
sudo dnf install mesa-libGLU-devel freeglut-devel -y  #安装GLU开发库
```
### Dora安装
```shell
pip install dora-rs-cli #安装dora命令行
wget https://github.com/dora-rs/dora/archive/refs/tags/v0.5.0.zip	#下载0.5.0版本的dora源码
cd dora/apis/c/node
cargo build --release   #编译
```
编译完成后可以在dora/target/release下看到libdora_node_api_c.a的链接库，说明编译成功。
然后在`NavigationFramework/third_party/`下新建`dora/lib`和`dora/include`，将`dora-0.5.0/target/release`下的`libdora_node_api_c.a`放到`lib`文件夹中，将`dora-0.5.0/apis/c/node`下的`node_api.h`放到`include`文件夹中。

### 第三方库
```shell
# Livox-SDK
cd third_party/Livox-SDK2
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
###########
# ndt_omp
cd third_party/ndt_omp
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
#########
# serial
cd third_party/serial
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
# 到沁恒管网下载CH340驱动 https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html
# 暂时由于系统内核版本原因，内核相关的内容是无法编译成功的，暂时先略过这部分。
unzip CH341SER.zip
cd CH341SER_LINUX/driver
make
sudo insmod ch341.ko
########
# g2o库
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
########
sudo dnf install nlohmann-json-devel -y #json库
sudo dnf install libpcap-devel -y  #pcap库(rslidar)
sudo dnf install asio-devel -y  #asio库(ranger底盘)
sudo dnf install glog-devel -y #(lightling-lm)
```
### rerun(源码编译安装)
```shell
#预先在系统中装好apache-arrow
wget https://github.com/apache/arrow/releases/download/apache-arrow-18.0.0/apache-arrow-18.0.0.tar.gz
tar -zxvf apache-arrow-18.0.0.tar.gz
cd apache-arrow-18.0.0/cpp
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DARROW_BUILD_TESTS=OFF \
  -DARROW_BUILD_EXAMPLES=OFF \
  -DARROW_BUILD_BENCHMARKS=OFF \
  -DARROW_FLIGHT=OFF \
  -DARROW_PYTHON=OFF \
  -DARROW_CUDA=OFF
make -j${nproc}
sudo make install
##########################
#接着再安装rerun
wget https://github.com/rerun-io/rerun/releases/download/0.31.2/rerun_cpp_sdk.zip   #下载rerun源码
unzip rerun_cpp_sdk.zip
cd rerun_cpp_sdk
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF
cmake --build build --config Release --target rerun_sdk
sudo cmake --install build
##########################
pip install rerun-sdk #安装命令行
```
### 部分文件夹需要先手动创建(预留为空，无法被git识别)
```shell
mkdir -p tools/map_trans/data/input tools/map_trans/data/output
mkdir -p third_party/dora/lib third_party/dora/include
mkdir -p maps/pcd maps/pgm
mkdir modules/mapping/maps
```
### 编译运行
```shell
mkdir build && cd build
cmake .. && make -j${nproc}
cd ..
dora run apps/xxx.yml #根据所需要的yml配置文件来选择
```
## 部分编译问题解决方法

### third_party/ndt_omp编译报错，没有VTK相关的包
openEuler 22.03 SP3系统仓库不提供这个包，最简单的解决方法：
`apps/align.cpp`代码中注释掉第10行的可视化头文件以及第107-114行的可视化代码，即可编译通过。
### tools/pgm_editor缺少CV库
官方源不提供 OpenCV 头文件，无法直接安装OpenCV，暂时无法使用该工具。
### lightning_lm_mapping找不到glog
打开`modules/mapping/lightning_lm_mapping/CMakeLists.txt`进行以下修改：
```shell
find_package(GLOG REQUIRED)替换为
				↓
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
########################################
新增link_directories(${GLOG_LIBRARY_DIRS})
########################################
两处链接库，把glog替换为${GLOG_LIBRARIES}
```

### 存在的问题

目前在OpenEuler系统上无法直接安装OpenCV，PCL库不支持可视化，无法使用pcl_viewer等工具。
目前系统内核没有加载CH340(串口芯片)、UTC2201(CAN芯片)，无法直接使用对应的通信接口来驱动设备。  
与外部设备通信这部分可以通过使用串口服务器等方式来尝试解决。