cmake_minimum_required(VERSION 3.5.1)

# Add sources.
file(GLOB libcarla_carla_sources
    "${libcarla_source_path}/carla/*.cpp"
    "${libcarla_source_path}/carla/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_sources}")
# install(FILES ${libcarla_carla_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla)

file(GLOB libcarla_carla_client_sources
    "${libcarla_source_path}/carla/client/*.cpp"
    "${libcarla_source_path}/carla/client/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_client_sources}")
# install(FILES ${libcarla_carla_client_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/client)

file(GLOB libcarla_carla_client_detail_sources
    "${libcarla_source_path}/carla/client/detail/*.cpp"
    "${libcarla_source_path}/carla/client/detail/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_client_detail_sources}")
# install(FILES ${libcarla_carla_client_detail_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/client/detail)

file(GLOB libcarla_carla_geom_sources
    "${libcarla_source_path}/carla/geom/*.cpp"
    "${libcarla_source_path}/carla/geom/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_geom_sources}")
# install(FILES ${libcarla_carla_geom_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/geom)

file(GLOB libcarla_carla_image_sources
    "${libcarla_source_path}/carla/image/*.cpp"
    "${libcarla_source_path}/carla/image/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_image_sources}")
# install(FILES ${libcarla_carla_image_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/image)

file(GLOB libcarla_carla_opendrive_sources
    "${libcarla_source_path}/carla/opendrive/*.cpp"
    "${libcarla_source_path}/carla/opendrive/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_opendrive_sources}")
# install(FILES ${libcarla_carla_opendrive_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/opendrive)

file(GLOB libcarla_carla_opendrive_parser_sources
    "${libcarla_source_path}/carla/opendrive/parser/*.cpp"
    "${libcarla_source_path}/carla/opendrive/parser/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_opendrive_parser_sources}")
# install(FILES ${libcarla_carla_opendrive_parser_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/opendrive/parser)

file(GLOB libcarla_carla_opendrive_parser_pugixml_sources
    "${libcarla_source_path}/carla/opendrive/parser/pugixml/*.cpp"
    "${libcarla_source_path}/carla/opendrive/parser/pugixml/*.hpp"
    "${libcarla_source_path}/carla/opendrive/parser/pugixml/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_opendrive_parser_pugixml_sources}")
# install(FILES ${libcarla_carla_opendrive_parser_pugixml_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/opendrive/parser/pugixml)

file(GLOB libcarla_carla_pointcloud_sources
    "${libcarla_source_path}/carla/pointcloud/*.cpp"
    "${libcarla_source_path}/carla/pointcloud/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_pointcloud_sources}")
# install(FILES ${libcarla_carla_pointcloud_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/pointcloud)

file(GLOB libcarla_carla_profiler_headers
    "${libcarla_source_path}/carla/profiler/*.h")
# install(FILES ${libcarla_carla_profiler_headers} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/profiler)

file(GLOB libcarla_carla_road_sources
    "${libcarla_source_path}/carla/road/*.cpp"
    "${libcarla_source_path}/carla/road/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_sources}")
# install(FILES ${libcarla_carla_road_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road)

file(GLOB libcarla_carla_road_element_sources
    "${libcarla_source_path}/carla/road/element/*.cpp"
    "${libcarla_source_path}/carla/road/element/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_element_sources}")
# install(FILES ${libcarla_carla_road_element_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road/element)

file(GLOB libcarla_carla_road_element_cephes_sources
    "${libcarla_source_path}/carla/road/element/cephes/*.cpp"
    "${libcarla_source_path}/carla/road/element/cephes/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_element_cephes_sources}")
# install(FILES ${libcarla_carla_road_element_cephes_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road/element/cephes)

file(GLOB libcarla_carla_road_general_sources
    "${libcarla_source_path}/carla/road/general/*.cpp"
    "${libcarla_source_path}/carla/road/general/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_general_sources}")
# install(FILES ${libcarla_carla_road_general_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road/general)

file(GLOB libcarla_carla_road_object_sources
    "${libcarla_source_path}/carla/road/object/*.cpp"
    "${libcarla_source_path}/carla/road/object/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_object_sources}")
# install(FILES ${libcarla_carla_road_object_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road/object)

file(GLOB libcarla_carla_road_signal_sources
    "${libcarla_source_path}/carla/road/signal/*.cpp"
    "${libcarla_source_path}/carla/road/signal/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_road_signal_sources}")
# install(FILES ${libcarla_carla_road_signal_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/road/signal)

file(GLOB libcarla_carla_rpc_sources
    "${libcarla_source_path}/carla/rpc/*.cpp"
    "${libcarla_source_path}/carla/rpc/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_rpc_sources}")
# install(FILES ${libcarla_carla_rpc_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/rpc)

file(GLOB libcarla_carla_sensor_sources
    "${libcarla_source_path}/carla/sensor/*.cpp"
    "${libcarla_source_path}/carla/sensor/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_sensor_sources}")
# install(FILES ${libcarla_carla_sensor_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/sensor)

file(GLOB libcarla_carla_sensor_data_sources
    "${libcarla_source_path}/carla/sensor/data/*.cpp"
    "${libcarla_source_path}/carla/sensor/data/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_sensor_data_sources}")
# install(FILES ${libcarla_carla_sensor_data_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/sensor/data)

file(GLOB libcarla_carla_sensor_s11n_sources
    "${libcarla_source_path}/carla/sensor/s11n/*.cpp"
    "${libcarla_source_path}/carla/sensor/s11n/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_sensor_s11n_sources}")
# install(FILES ${libcarla_carla_sensor_s11n_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/sensor/s11n)

file(GLOB libcarla_carla_streaming_sources
    "${libcarla_source_path}/carla/streaming/*.cpp"
    "${libcarla_source_path}/carla/streaming/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_streaming_sources}")
# install(FILES ${libcarla_carla_streaming_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/streaming)

file(GLOB libcarla_carla_streaming_detail_sources
    "${libcarla_source_path}/carla/streaming/detail/*.cpp"
    "${libcarla_source_path}/carla/streaming/detail/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_streaming_detail_sources}")
# install(FILES ${libcarla_carla_streaming_detail_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/streaming/detail)

file(GLOB libcarla_carla_streaming_detail_tcp_sources
    "${libcarla_source_path}/carla/streaming/detail/tcp/*.cpp"
    "${libcarla_source_path}/carla/streaming/detail/tcp/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_streaming_detail_tcp_sources}")
# install(FILES ${libcarla_carla_streaming_detail_tcp_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/streaming/detail/tcp)

file(GLOB libcarla_carla_streaming_low_level_sources
    "${libcarla_source_path}/carla/streaming/low_level/*.cpp"
    "${libcarla_source_path}/carla/streaming/low_level/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_carla_streaming_low_level_sources}")
# install(FILES ${libcarla_carla_streaming_low_level_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/carla/streaming/low_level)

file(GLOB libcarla_moodycamel_sources
    "${libcarla_source_path}/moodycamel/*.h")
set(libcarla_sources "${libcarla_sources};${libcarla_moodycamel_sources}")
# install(FILES ${libcarla_moodycamel_sources} DESTINATION ${LIBCARLA_HEADER_INSTALL_PATH}/include/lib/moodycamel)

# ==============================================================================
# Create targets for debug and release in the same build type.
# ==============================================================================

add_library(carla_client STATIC ${libcarla_sources})

target_include_directories(carla_client SYSTEM PRIVATE
    "${BOOST_INCLUDE_PATH}"
    "${RPCLIB_INCLUDE_PATH}")

# install(TARGETS carla_client DESTINATION ${LIBCARLA_LIB_INSTALL_PATH}/lib)

if (NOT DEFINED CMAKE_CXX_FLAGS_RELEASE_CLIENT)
  set(CMAKE_CXX_FLAGS_RELEASE_CLIENT ${CMAKE_CXX_FLAGS_RELEASE})
endif()
set_target_properties(carla_client PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE_CLIENT}")
