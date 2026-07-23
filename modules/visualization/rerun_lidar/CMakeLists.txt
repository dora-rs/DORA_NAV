project(rerun_lidar)

find_package(rerun_sdk REQUIRED)

include_directories(
    ${DORA_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    src/rerun_lidar.cpp
)

target_link_libraries(${PROJECT_NAME}
    rerun_sdk
    Threads::Threads
    ${DORA_NODE_API_LIB}
)

# Both rerun_sdk and dora_node_api_c are Rust static libraries that each bundle
# their own copy of the Rust std (rust_eh_personality, etc.).
# --allow-multiple-definition tells the linker to keep the first definition
# and silently ignore duplicates, resolving the symbol conflict.
target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,--allow-multiple-definition
)
