cmake_minimum_required(VERSION 3.5.1)

project(carlaviz-backend)
SET(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fPIC -O3 -DNDEBUG")

SET(CLIENT_ROOT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_BUILD_TYPE "Client")
include("third_party/LibCarla/cmake/CMakeLists.txt.in")
add_subdirectory("third_party/LibCarla/cmake")

add_subdirectory("third_party/xviz")
add_subdirectory("src/backend")