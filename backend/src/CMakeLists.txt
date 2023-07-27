find_package(gflags REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_library(
  carlaviz
  ${CARLAVIZ_SOURCE_DIR}/src/utils/logging.cc
  ${CARLAVIZ_SOURCE_DIR}/src/utils/map.cc
  ${CARLAVIZ_SOURCE_DIR}/src/utils/sensor.cc)

target_link_libraries(carlaviz gflags::gflags spdlog::spdlog fmt::fmt)

target_include_directories(
  carlaviz PUBLIC $<BUILD_INTERFACE:${CARLAVIZ_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)

message(STATUS "CarlaViz is using simulator ${CARLAVIZ_SIMULATOR}")
message(STATUS "CarlaViz is using frontend ${CARLAVIZ_FRONTEND}")

if(CARLAVIZ_SIMULATOR STREQUAL "CARLA")
  find_package(carla REQUIRED)
  target_link_libraries(carlaviz carla::client)
  target_compile_definitions(carlaviz PUBLIC CARLAVIZ_SIMULATOR_CARLA=1)
elseif()
  message(
    FATAL_ERROR "CarlaViz does not support simulator ${CARLAVIZ_SIMULATOR}")
endif()

if(CARLAVIZ_FRONTEND STREQUAL "XVIZ")
  find_package(xviz REQUIRED)
  find_package(websocketpp REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(lodepng REQUIRED)
  target_link_libraries(carlaviz xviz::xviz nlohmann_json::nlohmann_json
                        websocketpp::websocketpp lodepng::lodepng)
  target_compile_definitions(carlaviz PUBLIC CARLAVIZ_FRONTEND_XVIZ=1)
elseif()
  message(FATAL_ERROR "CarlaViz does not support frontend ${CARLAVIZ_FRONTEND}")
endif()

add_executable(backend backend.cc)
target_link_libraries(backend carlaviz)
if (MSVC)
  target_compile_options(backend PRIVATE /bigobj)
endif ()

