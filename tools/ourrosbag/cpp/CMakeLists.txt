cmake_minimum_required(VERSION 3.15)
project(ourrosbag_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

pybind11_add_module(ourrosbag_cpp parser.cpp)

install(TARGETS ourrosbag_cpp DESTINATION ${CMAKE_SOURCE_DIR}/../ourrosbag)