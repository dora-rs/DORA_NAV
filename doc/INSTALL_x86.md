## x86架构 22.04系统
### Dora安装
```shell
pip install dora-rs-cli #安装dora命令行
sudo apt install cargo  #安装cargo
rustup default stable   #编译可能报错，需要安装这个
git clone https://github.com/dora-rs/dora.git #克隆仓库 
cd dora/apis/c/node
cargo build --release   #编译
```
编译完成后可以在dora/target/release下看到libdora_node_api_c.a的链接库，说明编译成功。
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
```