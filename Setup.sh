#! /bin/bash

# ==============================================================================
# -- Set up environment --------------------------------------------------------
# ==============================================================================

command -v /usr/bin/clang++-7 >/dev/null 2>&1 || {
  echo >&2 "clang 7 is required, but it's not installed.";
  exit 1;
}

CXX_TAG=c7
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

source $(dirname "$0")/Environment.sh


# ==============================================================================
# -- mkdir include folder ------------------------------------------------------
# ==============================================================================
TMP_LIB_INCLUDE_PATH=${PWD}/include/lib
mkdir -p ${TMP_LIB_INCLUDE_PATH}

mkdir -p ${CARLA_BUILD_FOLDER}
pushd ${CARLA_BUILD_FOLDER} >/dev/null

# ==============================================================================
# -- Get boost includes --------------------------------------------------------
# ==============================================================================

BOOST_VERSION=1.69.0
BOOST_BASENAME="boost-${BOOST_VERSION}-${CXX_TAG}"

BOOST_INCLUDE=${TMP_LIB_INCLUDE_PATH}
#${PWD}/${BOOST_BASENAME}-install/include
BOOST_LIBPATH=${CARLA_BUILD_FOLDER}
#${PWD}/${BOOST_BASENAME}-install/lib

if [[ -d "${BOOST_BASENAME}-install" ]] ; then
  log "${BOOST_BASENAME} already installed."
else

  rm -Rf ${BOOST_BASENAME}-source

  BOOST_PACKAGE_BASENAME=boost_${BOOST_VERSION//./_}

  log "Retrieving boost."
  wget "https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz"

  log "Extracting boost for Python 2."
  tar -xzf ${BOOST_PACKAGE_BASENAME}.tar.gz
  mkdir -p ${BOOST_BASENAME}-install/include
  mv ${BOOST_PACKAGE_BASENAME} ${BOOST_BASENAME}-source

  pushd ${BOOST_BASENAME}-source >/dev/null

  BOOST_TOOLSET="gcc"
  BOOST_CFLAGS="-fPIC -std=c++14 -DBOOST_ERROR_CODE_HEADER_ONLY"

  ./bootstrap.sh \
      --with-toolset=gcc \
      --prefix=../boost-install \
      --with-libraries=filesystem

  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} stage release
  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} install
  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} --clean-all

  # Get rid of  python2 build artifacts completely & do a clean build for python3
  popd >/dev/null
  rm -Rf ${BOOST_BASENAME}-source
  rm ${BOOST_PACKAGE_BASENAME}.tar.gz

  cp -r ${BOOST_BASENAME}-install/include/boost ${BOOST_INCLUDE}/boost
  cp ${BOOST_BASENAME}-install/lib/* ${BOOST_LIBPATH}/ >/dev/null

  rm -Rf ${BOOST_BASENAME}-install

fi

unset BOOST_BASENAME

# ==============================================================================
# -- Get rpclib and compile it with libstdc++ -----------------------
# ==============================================================================

RPCLIB_PATCH=v2.2.1_c1
RPCLIB_BASENAME=rpclib-${RPCLIB_PATCH}-${CXX_TAG}

RPCLIB_LIBSTDCXX_INCLUDE=${TMP_LIB_INCLUDE_PATH}
#${PWD}/${RPCLIB_BASENAME}-libstdcxx-install/include
RPCLIB_LIBSTDCXX_LIBPATH=${CARLA_BUILD_FOLDER}
#${PWD}/${RPCLIB_BASENAME}-libstdcxx-install/lib

if [[ -d "${RPCLIB_BASENAME}-libstdcxx-install" ]] ; then
  log "${RPCLIB_BASENAME} already installed."
else
  rm -Rf \
      ${RPCLIB_BASENAME}-source \
      ${RPCLIB_BASENAME}-libstdcxx-build \
      ${RPCLIB_BASENAME}-libstdcxx-install

  log "Retrieving rpclib."

  git clone -b ${RPCLIB_PATCH} https://github.com/carla-simulator/rpclib.git ${RPCLIB_BASENAME}-source

  log "Building rpclib with libstdc++."

  mkdir -p ${RPCLIB_BASENAME}-libstdcxx-build

  pushd ${RPCLIB_BASENAME}-libstdcxx-build >/dev/null

  cmake -G "Unix Makefiles" \
      -DCMAKE_CXX_FLAGS="-fPIC -std=c++14" \
      -DCMAKE_INSTALL_PREFIX="../${RPCLIB_BASENAME}-libstdcxx-install" \
      ../${RPCLIB_BASENAME}-source

  make

  make install

  popd >/dev/null

  rm -Rf ${RPCLIB_BASENAME}-source ${RPCLIB_BASENAME}-libstdcxx-build

  cp -r ${RPCLIB_BASENAME}-libstdcxx-install/include/rpc ${RPCLIB_LIBSTDCXX_INCLUDE}/rpc
  cp -r ${RPCLIB_BASENAME}-libstdcxx-install/lib/* ${RPCLIB_LIBSTDCXX_LIBPATH}/ >/dev/null

  rm -Rf ${RPCLIB_BASENAME}-libstdcxx-install

fi

unset RPCLIB_BASENAME

# ==============================================================================
# -- Get GTest and compile it with libstdc++ --------------------------------------
# ==============================================================================

GTEST_VERSION=1.8.1
GTEST_BASENAME=gtest-${GTEST_VERSION}-${CXX_TAG}

GTEST_LIBSTDCXX_INCLUDE=${TMP_LIB_INCLUDE_PATH}
#${PWD}/${GTEST_BASENAME}-libstdcxx-install/include
GTEST_LIBSTDCXX_LIBPATH=${CARLA_BUILD_FOLDER}
#${PWD}/${GTEST_BASENAME}-libstdcxx-install/lib

if [[ -d "${GTEST_BASENAME}-libstdcxx-install" ]] ; then
  log "${GTEST_BASENAME} already installed."
else
  rm -Rf \
      ${GTEST_BASENAME}-source \
      ${GTEST_BASENAME}-libstdcxx-build \
      ${GTEST_BASENAME}-libstdcxx-install

  log "Retrieving Google Test."

  git clone --depth=1 -b release-${GTEST_VERSION} https://github.com/google/googletest.git ${GTEST_BASENAME}-source

  log "Building Google Test with libstdc++."

  mkdir -p ${GTEST_BASENAME}-libstdcxx-build

  pushd ${GTEST_BASENAME}-libstdcxx-build >/dev/null

  cmake -G "Unix Makefiles" \
      -DCMAKE_CXX_FLAGS="-std=c++14" \
      -DCMAKE_INSTALL_PREFIX="../${GTEST_BASENAME}-libstdcxx-install" \
      ../${GTEST_BASENAME}-source

  make

  make install

  popd >/dev/null

  rm -Rf ${GTEST_BASENAME}-source ${GTEST_BASENAME}-libstdcxx-build

  cp -r ${GTEST_BASENAME}-libstdcxx-install/include/gtest ${GTEST_LIBSTDCXX_INCLUDE}/gtest
  cp -r ${GTEST_BASENAME}-libstdcxx-install/lib/* ${GTEST_LIBSTDCXX_LIBPATH}/ >/dev/null

  rm -Rf ${GTEST_BASENAME}-libstdcxx-install

fi

unset GTEST_BASENAME

# ==============================================================================
# -- Generate Version.h --------------------------------------------------------
# ==============================================================================

CARLA_VERSION=$(get_git_repository_version)

log "CARLA version ${CARLA_VERSION}."

VERSION_H_FILE=${LIBCARLA_ROOT_FOLDER}/source/carla/Version.h
VERSION_H_FILE_GEN=${CARLA_BUILD_FOLDER}/Version.h

sed -e "s|\${CARLA_VERSION}|${CARLA_VERSION}|g" ${VERSION_H_FILE}.in > ${VERSION_H_FILE_GEN}

move_if_changed "${VERSION_H_FILE_GEN}" "${VERSION_H_FILE}"

# ==============================================================================
# -- Generate CMake config --------------------------------------
# ==============================================================================

log "Generating CMake configuration files."

# -- CMAKE_CONFIG_FILE ---------------------------------------------------------

cat >${CMAKE_CONFIG_FILE}.gen <<EOL
# Automatically generated by `basename "$0"`

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

# Uncomment to force support for an specific image format (require their
# respective libraries installed).
# add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT)
# add_definitions(-DLIBCARLA_IMAGE_WITH_JPEG_SUPPORT)
# add_definitions(-DLIBCARLA_IMAGE_WITH_TIFF_SUPPORT)

add_definitions(-DLIBCARLA_TEST_CONTENT_FOLDER="${LIBCARLA_TEST_CONTENT_FOLDER}")

set(BOOST_INCLUDE_PATH "${BOOST_INCLUDE}")

# Here libraries linking libstdc++.
set(RPCLIB_INCLUDE_PATH "${RPCLIB_LIBSTDCXX_INCLUDE}")
set(RPCLIB_LIB_PATH "${RPCLIB_LIBSTDCXX_LIBPATH}")
set(GTEST_INCLUDE_PATH "${GTEST_LIBSTDCXX_INCLUDE}")
set(GTEST_LIB_PATH "${GTEST_LIBSTDCXX_LIBPATH}")
set(BOOST_LIB_PATH "${BOOST_LIBPATH}")


EOL

if [ "${TRAVIS}" == "true" ] ; then
  log "Travis CI build detected: disabling PNG support."
  echo "add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=false)" >> ${CMAKE_CONFIG_FILE}.gen
else
  echo "add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true)" >> ${CMAKE_CONFIG_FILE}.gen
fi

# -- Move files ----------------------------------------------------------------

move_if_changed "${CMAKE_CONFIG_FILE}.gen" "${CMAKE_CONFIG_FILE}"

# ==============================================================================
# -- ...clean up ---------------------------------------------------------------
# ==============================================================================
rm -r cmake pkgconfig

popd >/dev/null

# ==============================================================================
# -- build lib carla -----------------------------------------------------------
# ==============================================================================

log "Begin to build carla library"

mkdir -p ${LIBCARLA_LIB_INSTALL_PATH}
mkdir -p ${LIBCARLA_HEADER_INSTALL_PATH}
mkdir -p ${LIBCARLA_BUILD_PATH}

echo "set(CMAKE_C_COMPILER /usr/bin/clang)" > ${LIBCARLA_BUILD_TOOLCHAIN}
echo "set(CMAKE_CXX_COMPILER /usr/bin/clang++)" >> ${LIBCARLA_BUILD_TOOLCHAIN}
echo "set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fPIC -O3 -DNDEBUG\" CACHE STRING \"\" FORCE)" >> ${LIBCARLA_BUILD_TOOLCHAIN}

pushd ${LIBCARLA_BUILD_PATH} >/dev/null

cmake \
  -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Client \
  -DLIBCARLA_BUILD_RELEASE=ON \
  -DLIBCARLA_BUILD_DEBUG=OFF \
  -DLIBCARLA_BUILD_TEST=OFF \
  -DCMAKE_TOOLCHAIN_FILE=${LIBCARLA_BUILD_TOOLCHAIN} \
  -DLIBCARLA_LIB_INSTALL_PATH=${LIBCARLA_LIB_INSTALL_PATH} \
  -DLIBCARLA_HEADER_INSTALL_PATH=${LIBCARLA_HEADER_INSTALL_PATH} \
  ${CARLA_ROOT_FOLDER}; \

make
make install



# ==============================================================================
# -- ...and we are done --------------------------------------------------------
# ==============================================================================
popd >/dev/null

log "Success!"
