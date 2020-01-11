cmake_minimum_required(VERSION 3.5.1)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()


include_directories(${XVIZ_ROOT_FOLDER}/include)
include_directories(${PROTOBUF_INCLUDE_DIRS})

add_library(xviz-pb
  proto/declarativeui.pb.cc
  proto/session.pb.cc
  proto/core.pb.cc
  proto/annotation.pb.cc
  proto/primitives.pb.cc
  proto/uiprimitives.pb.cc
  proto/style.pb.cc
  proto/options.pb.cc
  proto/envelope.pb.cc
)

target_link_libraries(xviz-pb ${PROTOBUF_LIBRARIES})

add_library(xviz-utils
  utils/utils.cpp
)

add_library(xviz-ui-builder
  builder/declarative_ui/ui_builder.cpp
  builder/declarative_ui/container_builder.cpp
  builder/declarative_ui/video_builder.cpp
  builder/declarative_ui/metric_builder.cpp
  builder/declarative_ui/base_ui_builder.cpp
)

target_link_libraries(xviz-ui-builder
  xviz-pb
)

add_library(xviz-builder
  builder/metadata.cpp
  builder/base_builder.cpp
  builder/pose.cpp
  builder/primitive.cpp
  builder/time_series.cpp
  builder/ui_primitive.cpp
  builder/xviz_builder.cpp
)

target_link_libraries(xviz-builder xviz-ui-builder)


add_library(xviz-message
  message.cpp
)

add_library(xviz)
target_link_libraries(xviz xviz-ui-builder xviz-message xviz-builder xviz-pb xviz-utils)
