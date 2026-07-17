cmake_minimum_required(VERSION 3.10)
project(pgm_editor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(pgm_editor
    src/main.cpp
    src/pgm_editor.cpp
)

target_link_libraries(pgm_editor ${OpenCV_LIBS})

target_compile_options(pgm_editor PRIVATE -Wall -Wextra)
