
cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)

include_directories(${CLIENT_ROOT_FOLDER}/include/lib ${CLIENT_ROOT_FOLDER}/include)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_library(xviz
  xviz/utils/metadata_stream.cpp
  xviz/utils/ui_config.cpp
  xviz/xviz_metadata_builder.cpp
  xviz/xviz_builder.cpp
  xviz/xviz_pose_builder.cpp
  xviz/xviz_primitive_builder.cpp
  xviz/xviz_time_series_builder.cpp
)


add_library(proxy-utils 
  utils/utils.cpp
  utils/base64.cpp
  utils/lodepng.cpp
)

add_library(proxy
  proxy/carla_proxy.cpp
  proxy/frontend_proxy.cpp
  proxy/drawing_proxy.cpp
 )


# add_executable(proxy proxy_starter.cpp)
add_executable(platform platform.cpp)

#target_link_libraries(carla-base carla_client rpc boost_filesystem)
target_link_libraries(proxy-utils carla_client rpc boost_filesystem)
#target_link_libraries(connector converter connector-utils websocket-server xviz)
target_link_libraries(proxy proxy-utils xviz)
# target_link_libraries(proxy proxy-lib)
target_link_libraries(platform proxy)
# target_link_libraries(converter connector-utils websocket-server)