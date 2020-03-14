
cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)


include_directories(${CLIENT_ROOT_FOLDER}/include)
include_directories(${CLIENT_ROOT_FOLDER}/include/system)
include_directories(${CLIENT_ROOT_FOLDER}/third_party/LibCarla/source)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_library(proxy-utils 
  utils/utils.cpp
  utils/lodepng.cpp
)

add_library(proxy-server
  server/carla_handler.cpp
  server/carla_session.cpp
)

target_link_libraries(proxy-server xviz)

add_library(proxy
  proxy/frontend_proxy.cpp
  proxy/carla_proxy.cpp
  proxy/drawing_proxy.cpp
)


add_executable(backend backend.cpp)

target_link_libraries(proxy-utils carla_client rpc boost_filesystem)
target_link_libraries(proxy proxy-utils proxy-server xviz)
target_link_libraries(backend proxy)