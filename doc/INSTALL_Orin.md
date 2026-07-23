**适用的环境是Jetson AGX Orin + Ubuntu20.04。**

## Dora安装

```shell
sudo apt install curl -y	#安装curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh	#安装rust
cargo install dora-cli #安装dora命令行
wget https://github.com/dora-rs/dora/archive/refs/tags/v0.5.0.zip	#下载0.5.0版本的dora源码
cd dora-0.5.0/apis/c/node
cargo build --release   #编译
```

编译完成后可以在dora/target/release下看到libdora_node_api_c.a的链接库，说明编译成功。

然后在`NavigationFramework/third_party/`下新建`dora/lib`和`dora/include`，将`dora-0.5.0/target/release`下的`libdora_node_api_c.a`放到`lib`文件夹中，将`dora-0.5.0/apis/c/node`下的`node_api.h`放到`include`文件夹中。

## 第三方库

```shell
# 开始之前可以先用鱼香ROS的命令安装ROS，这样系统就可以直接安装好PCL等环境，方便后续编译。
source <(wget -qO- http://fishros.com/install)
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
########
# g2o库
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
########
sudo apt install nlohmann-json3-dev #json库
sudo apt-get install -y  libpcap-dev  #pcap库(rslidar)
sudo apt install libasio-dev  #asio库(ranger底盘)
sudo apt-get install libgoogle-glog-dev #(lightling-lm)
```

## rerun(源码编译安装)
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
#如果上面这一步骤报错，Arrow::arrow_shared这个导入目标不是全局可见，在cmake的131行上方添加
#set_target_properties(Arrow::arrow_shared PROPERTIES IMPORTED_GLOBAL TRUE)
cmake --build build --config Release --target rerun_sdk
sudo cmake --install build
##########################

pip install rerun-sdk #安装命令行
#如果直接安装失败的话，可能是由于Ubuntu20的原因，python版本过低。
#安装conda，新建一个python>= 3.10的环境即可。
```

## 部分编译问题解决方法

### lightning_lm_mapping找不到glog

这好像是由于orin是arm架构的原因，sudo apt install安装后没有生成.cmake让find_package找到。

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

### rerun可视化节点报错，且内容和源码编译时的一致

在根目录CMAKE的find_package部分，显式查找 Arrow 并将其目标标记为全局：

```shell
find_package(Arrow REQUIRED)
if(TARGET Arrow::arrow_shared)
    set_target_properties(Arrow::arrow_shared PROPERTIES IMPORTED_GLOBAL TRUE)
endif()
```

### 链接库的问题

最简单的方法就是在每个节点的`target_link_libraries`中加上`rt dl`

另外，lightning-lim节点还需要单独加上`tbb`

### HDL定位节点报错

把`modules/localization/hdl_localization_dora/src/json_serialization.cpp`，`modules/localization/hdl_localization_dora/src/json_serialization.hpp`，`modules/localization/hdl_localization_dora/src/hdl_localization_dora_node.cpp`中std::make_shared<...>替换为boost::make_shared<...>。

同时`json_serialization.hpp`还需添加头文件`#include <boost/make_shared.hpp>`。

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