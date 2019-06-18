cmake_minimum_required(VERSION 3.5.1)
project(carla-client-example)

SET(CMAKE_C_COMPILER /usr/bin/gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
if (USE_CLANG)
    SET(CMAKE_C_COMPILER /usr/bin/clang)
    SET(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif(USE_CLANG)
unset(USE_CLANG CACHE)

add_subdirectory("src/example")