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

mkdir -p ${CARLA_BUILD_FOLDER}
pushd ${CARLA_BUILD_FOLDER} >/dev/null

# ==============================================================================
# -- Get boost includes --------------------------------------------------------
# ==============================================================================

BOOST_VERSION=1.69.0
BOOST_BASENAME="boost-${BOOST_VERSION}-${CXX_TAG}"

BOOST_INCLUDE=${PWD}/${BOOST_BASENAME}-install/include
BOOST_LIBPATH=${PWD}/${BOOST_BASENAME}-install/lib

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

  py2="/usr/bin/env python2"
  py2_root=`${py2} -c "import sys; print(sys.prefix)"`
  pyv=`$py2 -c "import sys;x='{v[0]}.{v[1]}'.format(v=list(sys.version_info[:2]));sys.stdout.write(x)";`
  ./bootstrap.sh \
      --with-toolset=gcc \
      --prefix=../boost-install \
      --with-libraries=python,filesystem \
      --with-python=${py2} --with-python-root=${py2_root}

  if ${TRAVIS} ; then
    echo "using python : ${pyv} : ${py2_root}/bin/python2 ;" > ${HOME}/user-config.jam
  else
    echo "using python : ${pyv} : ${py2_root}/bin/python2 ;" > project-config.jam
  fi

  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} stage release
  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} install
  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} --clean-all

  # Get rid of  python2 build artifacts completely & do a clean build for python3
  popd >/dev/null
  rm -Rf ${BOOST_BASENAME}-source

  log "Extracting boost for Python 3."
  tar -xzf ${BOOST_PACKAGE_BASENAME}.tar.gz
  mkdir -p ${BOOST_BASENAME}-install/include
  mv ${BOOST_PACKAGE_BASENAME} ${BOOST_BASENAME}-source

  pushd ${BOOST_BASENAME}-source >/dev/null

  py3="/usr/bin/env python3"
  py3_root=`${py3} -c "import sys; print(sys.prefix)"`
  pyv=`$py3 -c "import sys;x='{v[0]}.{v[1]}'.format(v=list(sys.version_info[:2]));sys.stdout.write(x)";`
  ./bootstrap.sh \
      --with-toolset=gcc \
      --prefix=../boost-install \
      --with-libraries=python \
      --with-python=${py3} --with-python-root=${py3_root}

  if ${TRAVIS} ; then
    echo "using python : ${pyv} : ${py3_root}/bin/python3 ;" > ${HOME}/user-config.jam
  else
    echo "using python : ${pyv} : ${py3_root}/bin/python3 ;" > project-config.jam
  fi

  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} stage release
  ./b2 toolset="${BOOST_TOOLSET}" cxxflags="${BOOST_CFLAGS}" --prefix="../${BOOST_BASENAME}-install" -j ${CARLA_BUILD_CONCURRENCY} install

  popd >/dev/null

  rm -Rf ${BOOST_BASENAME}-source
  rm ${BOOST_PACKAGE_BASENAME}.tar.gz

fi

unset BOOST_BASENAME

# ==============================================================================
# -- Get rpclib and compile it with libc++ and libstdc++ -----------------------
# ==============================================================================

RPCLIB_PATCH=v2.2.1_c1
RPCLIB_BASENAME=rpclib-${RPCLIB_PATCH}-${CXX_TAG}

RPCLIB_LIBCXX_INCLUDE=${PWD}/${RPCLIB_BASENAME}-libcxx-install/include
RPCLIB_LIBCXX_LIBPATH=${PWD}/${RPCLIB_BASENAME}-libcxx-install/lib
RPCLIB_LIBSTDCXX_INCLUDE=${PWD}/${RPCLIB_BASENAME}-libstdcxx-install/include
RPCLIB_LIBSTDCXX_LIBPATH=${PWD}/${RPCLIB_BASENAME}-libstdcxx-install/lib

if [[ -d "${RPCLIB_BASENAME}-libcxx-install" && -d "${RPCLIB_BASENAME}-libstdcxx-install" ]] ; then
  log "${RPCLIB_BASENAME} already installed."
else
  rm -Rf \
      ${RPCLIB_BASENAME}-source \
      ${RPCLIB_BASENAME}-libcxx-build ${RPCLIB_BASENAME}-libstdcxx-build \
      ${RPCLIB_BASENAME}-libcxx-install ${RPCLIB_BASENAME}-libstdcxx-install

  log "Retrieving rpclib."

  git clone -b ${RPCLIB_PATCH} https://github.com/carla-simulator/rpclib.git ${RPCLIB_BASENAME}-source

  log "Building rpclib with libstdc++."

  mkdir -p ${RPCLIB_BASENAME}-libstdcxx-build

  pushd ${RPCLIB_BASENAME}-libstdcxx-build >/dev/null

  cmake -G "Ninja" \
      -DCMAKE_CXX_FLAGS="-fPIC -std=c++14" \
      -DCMAKE_INSTALL_PREFIX="../${RPCLIB_BASENAME}-libstdcxx-install" \
      ../${RPCLIB_BASENAME}-source

  ninja

  ninja install

  popd >/dev/null

  rm -Rf ${RPCLIB_BASENAME}-source ${RPCLIB_BASENAME}-libstdcxx-build

fi

unset RPCLIB_BASENAME

# ==============================================================================
# -- Get GTest and compile it with libc++ --------------------------------------
# ==============================================================================

GTEST_VERSION=1.8.1
GTEST_BASENAME=gtest-${GTEST_VERSION}-${CXX_TAG}

GTEST_LIBCXX_INCLUDE=${PWD}/${GTEST_BASENAME}-libcxx-install/include
GTEST_LIBCXX_LIBPATH=${PWD}/${GTEST_BASENAME}-libcxx-install/lib
GTEST_LIBSTDCXX_INCLUDE=${PWD}/${GTEST_BASENAME}-libstdcxx-install/include
GTEST_LIBSTDCXX_LIBPATH=${PWD}/${GTEST_BASENAME}-libstdcxx-install/lib

if [[ -d "${GTEST_BASENAME}-libcxx-install" && -d "${GTEST_BASENAME}-libstdcxx-install" ]] ; then
  log "${GTEST_BASENAME} already installed."
else
  rm -Rf \
      ${GTEST_BASENAME}-source \
      ${GTEST_BASENAME}-libcxx-build ${GTEST_BASENAME}-libstdcxx-build \
      ${GTEST_BASENAME}-libcxx-install ${GTEST_BASENAME}-libstdcxx-install

  log "Retrieving Google Test."

  git clone --depth=1 -b release-${GTEST_VERSION} https://github.com/google/googletest.git ${GTEST_BASENAME}-source

  log "Building Google Test with libstdc++."

  mkdir -p ${GTEST_BASENAME}-libstdcxx-build

  pushd ${GTEST_BASENAME}-libstdcxx-build >/dev/null

  cmake -G "Ninja" \
      -DCMAKE_CXX_FLAGS="-std=c++14" \
      -DCMAKE_INSTALL_PREFIX="../${GTEST_BASENAME}-libstdcxx-install" \
      ../${GTEST_BASENAME}-source

  ninja

  ninja install

  popd >/dev/null

  rm -Rf ${GTEST_BASENAME}-source ${GTEST_BASENAME}-libstdcxx-build

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
# -- Generate CMake toolchains and config --------------------------------------
# ==============================================================================

log "Generating CMake configuration files."

# -- LIBSTDCPP_TOOLCHAIN_FILE --------------------------------------------------

cat >${LIBSTDCPP_TOOLCHAIN_FILE}.gen <<EOL
# Automatically generated by `basename "$0"`

set(CMAKE_C_COMPILER ${CC})
set(CMAKE_CXX_COMPILER ${CXX})

set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fPIC" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Wdeprecated -Wshadow -Wuninitialized -Wunreachable-code" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Wpessimizing-move -Wold-style-cast -Wnull-dereference" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Wduplicate-enum -Wnon-virtual-dtor -Wheader-hygiene" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Wconversion -Wfloat-overflow-conversion" CACHE STRING "" FORCE)

# @todo These flags need to be compatible with setup.py compilation.
set(CMAKE_CXX_FLAGS_RELEASE_CLIENT "\${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -g -fwrapv -O2 -Wall -Wstrict-prototypes -fno-strict-aliasing -Wdate-time -D_FORTIFY_SOURCE=2 -g -fstack-protector-strong -Wformat -Werror=format-security -fPIC -std=c++14 -Wno-missing-braces -DBOOST_ERROR_CODE_HEADER_ONLY -DLIBCARLA_WITH_PYTHON_SUPPORT" CACHE STRING "" FORCE)
EOL

# -- LIBCPP_TOOLCHAIN_FILE -----------------------------------------------------

# We can reuse the previous toolchain.
cp ${LIBSTDCPP_TOOLCHAIN_FILE}.gen ${LIBCPP_TOOLCHAIN_FILE}.gen

cat >>${LIBCPP_TOOLCHAIN_FILE}.gen <<EOL

set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -stdlib=libc++" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -isystem ${LLVM_INCLUDE}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti" CACHE STRING "" FORCE)
set(CMAKE_CXX_LINK_FLAGS "\${CMAKE_CXX_LINK_FLAGS} -L${LLVM_LIBPATH}" CACHE STRING "" FORCE)
set(CMAKE_CXX_LINK_FLAGS "\${CMAKE_CXX_LINK_FLAGS} -lc++ -lc++abi" CACHE STRING "" FORCE)
EOL

# -- CMAKE_CONFIG_FILE ---------------------------------------------------------

cat >${CMAKE_CONFIG_FILE}.gen <<EOL
# Automatically generated by `basename "$0"`

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

if (CMAKE_BUILD_TYPE STREQUAL "Server")
  add_definitions(-DASIO_NO_EXCEPTIONS)
  add_definitions(-DBOOST_NO_EXCEPTIONS)
  add_definitions(-DLIBCARLA_NO_EXCEPTIONS)
  add_definitions(-DPUGIXML_NO_EXCEPTIONS)
endif ()

# Uncomment to force support for an specific image format (require their
# respective libraries installed).
# add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT)
# add_definitions(-DLIBCARLA_IMAGE_WITH_JPEG_SUPPORT)
# add_definitions(-DLIBCARLA_IMAGE_WITH_TIFF_SUPPORT)

add_definitions(-DLIBCARLA_TEST_CONTENT_FOLDER="${LIBCARLA_TEST_CONTENT_FOLDER}")

set(BOOST_INCLUDE_PATH "${BOOST_INCLUDE}")

if (CMAKE_BUILD_TYPE STREQUAL "Server")
  # Here libraries linking libc++.
  set(LLVM_INCLUDE_PATH "${LLVM_INCLUDE}")
  set(LLVM_LIB_PATH "${LLVM_LIBPATH}")
  set(RPCLIB_INCLUDE_PATH "${RPCLIB_LIBCXX_INCLUDE}")
  set(RPCLIB_LIB_PATH "${RPCLIB_LIBCXX_LIBPATH}")
  set(GTEST_INCLUDE_PATH "${GTEST_LIBCXX_INCLUDE}")
  set(GTEST_LIB_PATH "${GTEST_LIBCXX_LIBPATH}")
elseif (CMAKE_BUILD_TYPE STREQUAL "Client")
  # Here libraries linking libstdc++.
  set(RPCLIB_INCLUDE_PATH "${RPCLIB_LIBSTDCXX_INCLUDE}")
  set(RPCLIB_LIB_PATH "${RPCLIB_LIBSTDCXX_LIBPATH}")
  set(GTEST_INCLUDE_PATH "${GTEST_LIBSTDCXX_INCLUDE}")
  set(GTEST_LIB_PATH "${GTEST_LIBSTDCXX_LIBPATH}")
  set(BOOST_LIB_PATH "${BOOST_LIBPATH}")
endif ()

EOL

if [ "${TRAVIS}" == "true" ] ; then
  log "Travis CI build detected: disabling PNG support."
  echo "add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=false)" >> ${CMAKE_CONFIG_FILE}.gen
else
  echo "add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true)" >> ${CMAKE_CONFIG_FILE}.gen
fi

# -- Move files ----------------------------------------------------------------

move_if_changed "${LIBSTDCPP_TOOLCHAIN_FILE}.gen" "${LIBSTDCPP_TOOLCHAIN_FILE}"
move_if_changed "${LIBCPP_TOOLCHAIN_FILE}.gen" "${LIBCPP_TOOLCHAIN_FILE}"
move_if_changed "${CMAKE_CONFIG_FILE}.gen" "${CMAKE_CONFIG_FILE}"

# ==============================================================================
# -- ...and we are done --------------------------------------------------------
# ==============================================================================

popd >/dev/null

log "Success!"
