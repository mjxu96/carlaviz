cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_C_COMPILER /usr/bin/gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -fPIC -O3 -DNDEBUG")
if (USE_CLANG)
    SET(CMAKE_C_COMPILER /usr/bin/clang)
    SET(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif(USE_CLANG)
unset(USE_CLANG CACHE)

SET(CLIENT_ROOT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory("src/libcarla/cmake")
add_subdirectory("src/example")