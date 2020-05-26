#! /bin/bash

# Sets the environment for other shell scripts.

set -e

# ==============================================================================
# -- Environment variable definitions ------------------------------------------
# ==============================================================================

# Here CARLA_FOLDER is assumed to be the root folder of the project.

CARLA_ROOT_FOLDER=${CARLA_FOLDER}
CARLA_BUILD_FOLDER=${CARLA_FOLDER}/lib

# LIBCARLA_ROOT_FOLDER=${CARLA_FOLDER}/src/libcarla


CMAKE_CONFIG_FILE=${CARLA_FOLDER}/third_party/LibCarla/cmake/CMakeLists.txt.in

LIBCARLA_TEST_CONTENT_FOLDER=${CARLA_BUILD_FOLDER}/test-content

# libcarla variables

LIBCARLA_BUILD_PATH=${CARLA_FOLDER}/build
LIBCARLA_LIB_INSTALL_PATH=${CARLA_FOLDER}
LIBCARLA_HEADER_INSTALL_PATH=${CARLA_FOLDER}
LIBCARLA_BUILD_TOOLCHAIN=${CARLA_FOLDER}/src/libcarla/cmake/toolchain.cmake

LIB_BUILD_CONCURRENCY=$(nproc --all)
CARLA_BUILD_CONCURRENCY=`nproc --all`

# ==============================================================================
# -- Useful function definitions -----------------------------------------------
# ==============================================================================

function log {
  echo -e "\033[1;35m`basename "$0"`: $1\033[0m"
}

function fatal_error {
  echo -e >&2 "\033[0;31m`basename "$0"`: ERROR: $1\033[0m"
  exit 2
}


function get_git_repository_version {
  # git describe --tags --dirty --always
  echo "0.9.9"
}

function copy_if_changed {
  mkdir -p $(dirname $2)
  rsync -cIr --out-format="%n" $1 $2
}

function move_if_changed {
  copy_if_changed $1 $2
  rm -f $1
}

