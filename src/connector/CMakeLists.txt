
cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)

include_directories(${CLIENT_ROOT_FOLDER}/include/lib ${CLIENT_ROOT_FOLDER}/include)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_library(xviz
  xviz/utils/metadata_stream.cpp
  xviz/xviz_metadata_builder.cpp
  xviz/xviz_builder.cpp
  xviz/xviz_pose_builder.cpp
  xviz/xviz_primitive_builder.cpp
  xviz/xviz_time_series_builder.cpp
)

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
add_executable(proxy proxy.cpp)


target_link_libraries(connector-utils carla_client rpc boost_filesystem)
target_link_libraries(connector converter connector-utils websocket-server xviz)
target_link_libraries(proxy converter connector-utils websocket-server xviz)
# target_link_libraries(converter connector-utils websocket-server)