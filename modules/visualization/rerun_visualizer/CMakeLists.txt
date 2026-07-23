project(rerun_visualizer)

find_package(rerun_sdk REQUIRED)

include_directories(
    ${DORA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/rerun_visualizer.cpp
)

target_link_libraries(${PROJECT_NAME}
    nlohmann_json::nlohmann_json
    rerun_sdk
    ${PCL_LIBRARIES}
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
