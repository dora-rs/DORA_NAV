#!/bin/bash
set -e

NAV_ROOT="$(cd "$(dirname "$0")" && pwd)"
DORA_ROOT="$HOME/dora"
DORA_INCLUDE="$DORA_ROOT/apis/c/node"
DORA_OPERATOR="$DORA_ROOT/apis/c/operator"
DORA_LIB="$DORA_ROOT/target/release/libdora_node_api_c.a"

echo "=== Building all dora-nav nodes ==="
echo "NAV_ROOT: $NAV_ROOT"
echo "DORA_LIB: $DORA_LIB"

FAILED=()
BUILT=()

build_cmake_node() {
    local dir="$1"
    local name="$2"
    echo ""
    echo "--- Building $name ($dir) ---"

    mkdir -p "$NAV_ROOT/$dir/build"
    cd "$NAV_ROOT/$dir/build"

    if cmake .. \
        -DCMAKE_PREFIX_PATH="$NAV_ROOT/cmake" \
        -DDORA_INCLUDE_DIR="$DORA_INCLUDE" \
        -DDORA_OPERATOR_DIR="$DORA_OPERATOR" \
        -DDORA_LIB_PATH="$DORA_LIB" \
        -DDORA_NAV_ROOT="$NAV_ROOT" \
        2>&1; then
        if make -j$(nproc) 2>&1; then
            BUILT+=("$name")
            echo "--- $name: BUILD SUCCESS ---"
        else
            FAILED+=("$name")
            echo "--- $name: BUILD FAILED (make) ---"
        fi
    else
        FAILED+=("$name")
        echo "--- $name: BUILD FAILED (cmake) ---"
    fi
    cd "$NAV_ROOT"
}

# Build each node
build_cmake_node "driver/rslidar_driver" "rslidar_driver"
build_cmake_node "mapping/test_rerun_mapping" "test_rerun_mapping"
build_cmake_node "mapping/ndt_mapping"   "ndt_mapping"
# build_cmake_node "map/pub_road" "pubroad"
# build_cmake_node "planning/mission_planning/task_pub" "task_pub_node"
# build_cmake_node "control/vehicle_control/lon_controller" "lon_controller_node"
# build_cmake_node "map/road_line_publisher" "road_lane_publisher_node"
# build_cmake_node "planning/routing_planning" "routing_planning_node"
# build_cmake_node "control/vehicle_control/lat_controller" "lat_controller_node"
#build_cmake_node "localization/dora-hdl_localization" "hdl_localization"

echo ""
echo "=== Build Summary ==="
echo "Built: ${BUILT[*]}"
echo "Failed: ${FAILED[*]}"
