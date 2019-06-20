cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)

include_directories(${CLIENT_ROOT_FOLDER}/include ${CLIENT_ROOT_FOLDER}/include/lib)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_executable(main main.cpp)

target_link_libraries(main carla_client rpc boost_filesystem png tiff jpeg)