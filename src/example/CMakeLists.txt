cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -fPIC -O3 -DNDEBUG")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../bin)

include_directories(../../include/lib)
link_directories(../../lib)

add_executable(main main.cpp)

target_link_libraries(main carla_client rpc boost_filesystem png tiff jpeg)