# cmake_minimum_required(VERSION 3.5)
# project(serial)

# find_package(ament_cmake REQUIRED)

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

# ## Sources
# ## Add serial library
# add_library(${PROJECT_NAME} SHARED
#     src/serial.cc
#     include/serial/serial.h
#     include/serial/v8stdint.h
# )

# if(APPLE) # macOS
#     find_library(IOKIT_LIBRARY IOKit)
#     find_library(FOUNDATION_LIBRARY Foundation)
#     target_sources(${PROJECT_NAME} PRIVATE
#         src/impl/unix.cc
#         src/impl/list_ports/list_ports_osx.cc
#     )
#      target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
# elseif(UNIX) # .*nix
#     target_sources(${PROJECT_NAME} PRIVATE
#         src/impl/unix.cc
#         src/impl/list_ports/list_ports_linux.cc
#      )
#          target_link_libraries(${PROJECT_NAME} rt pthread)
# elseif(WIN32) # Windows
#     target_sources(${PROJECT_NAME} PRIVATE
#         src/impl/win.cc
#         src/impl/list_ports/list_ports_win.cc
#     )
#       target_link_libraries(${PROJECT_NAME} setupapi)
#     ament_export_libraries(setupapi)
# endif()


# ## Include headers
# target_include_directories(${PROJECT_NAME} PRIVATE include)

# ## Uncomment for example
# # add_executable(serial_example examples/serial_example.cc)
# # add_dependencies(serial_example ${PROJECT_NAME})
# # target_link_libraries(serial_example ${PROJECT_NAME})

# ## Install executable
# install(TARGETS ${PROJECT_NAME}
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
# )

# ## Install headers
# install(FILES include/serial/serial.h include/serial/v8stdint.h
#     DESTINATION include/serial
# )

# ## Tests
# #if(BUILD_TESTING)
# #    add_subdirectory(tests)
# #endif()

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(serial)

# 设置 C++ 标准（兼容原代码）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 源文件与库配置
add_library(${PROJECT_NAME} SHARED
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

# 系统适配：根据操作系统链接对应依赖
if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
    )
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # Linux（含 Debian 12）
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
    )
    target_link_libraries(${PROJECT_NAME} rt pthread) # Linux 串口依赖的系统库
elseif(WIN32) # Windows
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# 头文件目录（让编译器找到 include 下的头文件）
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 编译时
        $<INSTALL_INTERFACE:include> # 安装后
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include # 库内部使用
)

# 安装配置（安装到系统默认目录 /usr/local）
# 安装库文件（动态库 .so）
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(FILES 
    include/serial/serial.h 
    include/serial/v8stdint.h
    DESTINATION include/serial
)

# 可选：编译示例程序（取消注释即可使用）
# add_executable(serial_example examples/serial_example.cc)
# target_link_libraries(serial_example ${PROJECT_NAME})
# install(TARGETS serial_example DESTINATION bin)
