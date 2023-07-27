cmake_minimum_required(VERSION 3.14)

if(NOT CARLAVIZ_VERSION)
  set(CARLAVIZ_VERSION "l.0.0")
endif()

project(
  carlaviz
  VERSION ${CARLAVIZ_VERSION}
  DESCRIPTION "CarlaViz"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(CARLAVIZ_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory("src")
