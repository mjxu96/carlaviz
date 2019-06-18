cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_C_COMPILER /usr/bin/clang)
SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

add_executable(main main.cpp)

include_directories(../../include/lib)
link_directories(../../lib)

target_link_libraries(main carla_client rpc boost_filesystem png tiff jpeg)