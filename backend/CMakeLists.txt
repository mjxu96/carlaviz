cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_C_COMPILER /usr/bin/gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
if (USE_CLANG)
    SET(CMAKE_C_COMPILER /usr/bin/clang)
    SET(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif(USE_CLANG)
unset(USE_CLANG CACHE)

project(carla-display-backend)
SET(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fPIC -O3 -DNDEBUG")

SET(CLIENT_ROOT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

set(LIBCARLA_BUILD_DEBUG OFF)
set(LIBCARLA_BUILD_TEST OFF)
set(CMAKE_INSTALL_PREFIX ${CLIENT_ROOT_FOLDER})
set(CMAKE_BUILD_TYPE "Client")
include("third_party/LibCarla/cmake/CMakeLists.txt.in")
add_subdirectory("third_party/LibCarla/cmake")

add_subdirectory("third_party/xviz")
add_subdirectory("src/backend")