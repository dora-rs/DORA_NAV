## x86架构 22.04系统
### Dora安装
```shell
# 1. 安装 dora 命令行（1.0 的 dora-rs-cli 需要 Python 3.11+；pip 装不上就改用 cargo）
pip install -U dora-rs-cli            # 或：cargo install dora-cli --locked
sudo apt install cargo  #安装cargo
# 2. 编译 dora C 节点链接库（dora 1.0 需要 Rust >= 1.95）
rustup default stable   #编译可能报错，需要安装这个
wget https://github.com/dora-rs/dora/archive/refs/tags/v1.0.0.zip #下载1.0.0版本的dora源码
cd dora-1.0.0/apis/c/node
cargo build --release   #编译
#编译完成后可以在dora-1.0.0/target/release下看到libdora_node_api_c.a的链接库，说明编译成功。
# 3. 把编译产物拷贝到工程（CMake 会从 third_party/dora 读取）
mkdir -p third_party/dora/lib third_party/dora/include
cp dora-1.0.0/target/release/libdora_node_api_c.a third_party/dora/lib/
cp dora-1.0.0/apis/c/node/node_api.h third_party/dora/include/
```
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
# 重启串口
sudo apt remove brltty
sudo systemctl stop brltty
sudo systemctl disable brltty
########
# g2o库
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install  #如果 -j${nproc}报问题就手动指定核心数即可
########
sudo apt install nlohmann-json3-dev #json库
sudo apt-get install -y  libpcap-dev  #pcap库(rslidar)
sudo apt install libasio-dev  #asio库(ranger底盘)
sudo apt-get install libgoogle-glog-dev #(lightling-lm)
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
# dora run 会在本机隔离运行数据流（无需先 dora up，但没有 dora stop / dora logs 管理）。
# 如需管理（list/stop/logs），改用协调模式：
#   dora up
#   dora start apps/xxx.yml --detach     # 后台运行
#   dora logs apps/xxx.yml --node <名字>  # 查看某节点日志
#   dora stop <名字或uuid>
#   dora down
```