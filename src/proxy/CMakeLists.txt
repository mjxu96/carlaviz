
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


add_library(proxy-utils 
  utils/utils.cpp
)

add_library(proxy proxy.cpp)


add_executable(proxy_starter, proxy_starter.cpp)

#target_link_libraries(carla-base carla_client rpc boost_filesystem)
target_link_libraries(proxy-utils carla_client rpc boost_filesystem)
#target_link_libraries(connector converter connector-utils websocket-server xviz)
target_link_libraries(proxy proxy-utils xviz)
target_link_libraries(proxy_starter proxy)
# target_link_libraries(converter connector-utils websocket-server)