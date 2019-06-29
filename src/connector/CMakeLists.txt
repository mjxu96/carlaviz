
cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)

include_directories(${CLIENT_ROOT_FOLDER}/include/lib ${CLIENT_ROOT_FOLDER}/include)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_library(connector-utils
  utils/package.cpp
)

add_library(websocket-server
  websocket_server.cpp
)

add_library(converter 
  utils/xodr_geojson_converter.cpp
)

add_executable(connector connector.cpp)


target_link_libraries(connector-utils carla_client rpc boost_filesystem)
target_link_libraries(connector converter connector-utils websocket-server)
# target_link_libraries(converter connector-utils websocket-server)