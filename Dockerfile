FROM ubuntu:22.04

# Prevent interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    build-essential clang cmake git curl wget unzip \
    libpcl-dev libeigen3-dev libboost-all-dev libyaml-cpp-dev \
    nlohmann-json3-dev \
    python3-dev python3-pip \
    libgl1-mesa-glx libgl1-mesa-dri \
    libglib2.0-0 libxext6 libxrender1 libxcb1 \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# ── Python packages ─────────────────────────────────────────────────────────────
# IMPORTANT: pin rerun-sdk to 0.29.2 to match the C++ SDK built below
RUN pip3 install --no-cache-dir mujoco dora-rs-cli rerun-sdk==0.29.2

# ── Rust + Cargo (needed to build DORA C API) ─────────────────────────────────
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# ── Build DORA C API library ────────────────────────────────────────────────────
# FIX: clone into /root/Public/dora so dora_config.cmake finds it at $HOME/Public/dora
RUN mkdir -p /root/Public && \
    cd /root/Public && \
    git clone https://github.com/dora-rs/dora.git && \
    cd dora && \
    cargo build -p dora-node-api-c --release

# Convenience ENV vars reused by all cmake build steps below
ENV DORA_INCLUDE=/root/Public/dora/apis/c/node
ENV DORA_OPERATOR=/root/Public/dora/apis/c/operator
ENV DORA_LIB=/root/Public/dora/target/release/libdora_node_api_c.a
ENV NAV_ROOT=/root/Public/dora-nav

# ── Build Rerun C++ SDK ─────────────────────────────────────────────────────────
# rerun/CMakeLists.txt hardcodes path: /tmp/rerun_cpp_sdk/rerun_cpp_sdk/build
# The zip extracts a rerun_cpp_sdk/ subfolder → unzip into /tmp/rerun_cpp_sdk/
# so it lands at /tmp/rerun_cpp_sdk/rerun_cpp_sdk/CMakeLists.txt ✓
RUN mkdir -p /tmp/rerun_cpp_sdk && \
    cd /tmp/rerun_cpp_sdk && \
    wget -q https://github.com/rerun-io/rerun/releases/download/0.29.2/rerun_cpp_sdk.zip && \
    unzip -q rerun_cpp_sdk.zip && \
    mkdir -p rerun_cpp_sdk/build && \
    cd rerun_cpp_sdk/build && cmake .. && make -j$(nproc)

# ── Copy DORA_NAV source ────────────────────────────────────────────────────────
WORKDIR /root/Public/dora-nav
COPY . .

# Fix any remaining hardcoded developer paths in dataflow configs
RUN sed -i 's|/home/demo/Public/dora-nav|/root/Public/dora-nav|g' \
    dataflow_full_sim.yml run.yml 2>/dev/null || true

# ── Build each node explicitly (so failures are visible) ───────────────────────

RUN cd map/pub_road && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd planning/mission_planning/task_pub && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd control/vehicle_control/lon_controller && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd map/road_line_publisher && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd planning/routing_planning && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd control/vehicle_control/lat_controller && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

# ── Build ndt_omp static library (no ROS, so we compile sources directly) ──────
RUN NDT_DIR=/root/Public/dora-nav/localization/dora-hdl_localization/3rdparty/hdl_ndt_omp && \
    PCL_INCLUDES=$(pkg-config --cflags-only-I eigen3 2>/dev/null || true) && \
    PCL_INC_DIR=$(find /usr/include -name "pcl" -maxdepth 3 -type d | head -1 | xargs dirname 2>/dev/null || echo "/usr/include") && \
    mkdir -p ${NDT_DIR}/build && \
    g++ -std=c++14 -O2 -fPIC \
        -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 \
        -fopenmp \
        -I${NDT_DIR}/include \
        -I${PCL_INC_DIR} \
        ${PCL_INCLUDES} \
        -c ${NDT_DIR}/src/pclomp/voxel_grid_covariance_omp.cpp -o ${NDT_DIR}/build/voxel_grid_covariance_omp.o && \
    g++ -std=c++14 -O2 -fPIC \
        -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 \
        -fopenmp \
        -I${NDT_DIR}/include \
        -I${PCL_INC_DIR} \
        ${PCL_INCLUDES} \
        -c ${NDT_DIR}/src/pclomp/ndt_omp.cpp -o ${NDT_DIR}/build/ndt_omp.o && \
    g++ -std=c++14 -O2 -fPIC \
        -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 \
        -fopenmp \
        -I${NDT_DIR}/include \
        -I${PCL_INC_DIR} \
        ${PCL_INCLUDES} \
        -c ${NDT_DIR}/src/pclomp/gicp_omp.cpp -o ${NDT_DIR}/build/gicp_omp.o && \
    ar rcs ${NDT_DIR}/build/libndt_omp.a \
        ${NDT_DIR}/build/voxel_grid_covariance_omp.o \
        ${NDT_DIR}/build/ndt_omp.o \
        ${NDT_DIR}/build/gicp_omp.o && \
    echo "libndt_omp.a built successfully"

RUN cd localization/dora-hdl_localization && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd simulation/mujoco_bridge && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

RUN cd rerun && mkdir -p build && cd build && \
    cmake .. -DDORA_INCLUDE_DIR=$DORA_INCLUDE -DDORA_OPERATOR_DIR=$DORA_OPERATOR \
    -DDORA_LIB_PATH=$DORA_LIB -DDORA_NAV_ROOT=$NAV_ROOT && \
    make -j$(nproc)

# ── Verify all binaries exist (build fails here if any are missing) ─────────────
RUN echo "=== Verifying all binaries ===" && \
    ls -la simulation/mujoco_bridge/build/mujoco_sim_bridge && \
    ls -la map/pub_road/build/pubroad && \
    ls -la map/road_line_publisher/build/road_lane_publisher_node && \
    ls -la planning/mission_planning/task_pub/build/task_pub_node && \
    ls -la planning/routing_planning/build/routing_planning_node && \
    ls -la control/vehicle_control/lat_controller/build/lat_controller_node && \
    ls -la control/vehicle_control/lon_controller/build/lon_controller_node && \
    ls -la rerun/build/to_rerun && \
    echo "=== All 8 binaries OK ==="

# ── Default command: run the full simulation ─────────────────────────────────────
CMD ["bash", "-c", "dora up && dora start /root/Public/dora-nav/dataflow_full_sim.yml --attach"]
