cmake_minimum_required(VERSION 3.5.1)
project(CARLA)

include("lib/CMakeLists.txt.in")

add_subdirectory("src/libcarla/cmake")
