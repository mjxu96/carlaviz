#! /bin/bash

# ==============================================================================
# -- Set up environment --------------------------------------------------------
# ==============================================================================


if ! [ -x "$(command -v /usr/bin/clang++-7)" ] && ! [ -x "$(command -v /usr/bin/g++-7)" ]; then
  echo >&2 "clang 7 or gcc 7 is required, but they're not installed.";
  exit 1;
fi

CARLA_FOLDER=$(cd "$(dirname "$0")"; cd ../; pwd)
source ${CARLA_FOLDER}/setup/environment.sh
unset CARLA_FOLDER

log "clean up previous build files"
rm -rf ${CARLA_BUILD_FOLDER} ${CARLA_ROOT_FOLDER}/build
rm -rf ${CARLA_ROOT_FOLDER}/include/lib/boost ${CARLA_ROOT_FOLDER}/include/lib/rpc ${CARLA_ROOT_FOLDER}/include/lib/gtest ${CARLA_ROOT_FOLDER}/include/lib/recast
rm -rf ${CARLA_ROOT_FOLDER}/include/system
rm -rf ${CARLA_ROOT_FOLDER}/include/lib
rm -f ${CMAKE_CONFIG_FILE}

CXX_TAG=c7
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
if [[ $1 = "clang" ]]; then
  export CC=/usr/bin/clang
  export CXX=/usr/bin/clang++
  log "using clang"
else
  log "using gcc"
fi

# ==============================================================================
# -- mkdir include folder ------------------------------------------------------
# ==============================================================================
LIB_HEADER_INCLUDE_PATH=${CARLA_ROOT_FOLDER}/include/system
mkdir -p ${LIB_HEADER_INCLUDE_PATH}

mkdir -p ${CARLA_BUILD_FOLDER}
mkdir -p ${CARLA_ROOT_FOLDER}/include/system
pushd ${CARLA_BUILD_FOLDER} >/dev/null

# ==============================================================================
# -- Get boost includes --------------------------------------------------------
# ==============================================================================

BOOST_VERSION=1.72.0
BOOST_BASENAME="boost-${BOOST_VERSION}-${CXX_TAG}"

BOOST_INCLUDE=${LIB_HEADER_INCLUDE_PATH}
BOOST_LIBPATH=${CARLA_BUILD_FOLDER}

rm -Rf ${BOOST_BASENAME}-source

BOOST_PACKAGE_BASENAME=boost_${BOOST_VERSION//./_}

log "Retrieving boost."
wget "https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz" || true
# try to use the backup boost we have in Jenkins
if [[ ! -f "${BOOST_PACKAGE_BASENAME}.tar.gz" ]] ; then
  log "Using boost backup"
  wget "https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/${BOOST_PACKAGE_BASENAME}.tar.gz" || true
fi

log "Building boost filesystem object file."
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
cp -r ${BOOST_BASENAME}-install/lib/* ${BOOST_LIBPATH}/ >/dev/null

rm -Rf ${BOOST_BASENAME}-install


unset BOOST_BASENAME

# ==============================================================================
# -- Get rpclib and compile it with libstdc++ -----------------------
# ==============================================================================

RPCLIB_PATCH=v2.2.1_c3
RPCLIB_BASENAME=rpclib-${RPCLIB_PATCH}-${CXX_TAG}

RPCLIB_LIBSTDCXX_INCLUDE=${LIB_HEADER_INCLUDE_PATH}
#${PWD}/${RPCLIB_BASENAME}-libstdcxx-install/include
RPCLIB_LIBSTDCXX_LIBPATH=${CARLA_BUILD_FOLDER}
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

make -j${LIB_BUILD_CONCURRENCY}

make install

popd >/dev/null

rm -Rf ${RPCLIB_BASENAME}-source ${RPCLIB_BASENAME}-libstdcxx-build

cp -r ${RPCLIB_BASENAME}-libstdcxx-install/include/rpc ${RPCLIB_LIBSTDCXX_INCLUDE}/rpc
cp -r ${RPCLIB_BASENAME}-libstdcxx-install/lib/* ${RPCLIB_LIBSTDCXX_LIBPATH}/ >/dev/null

rm -Rf ${RPCLIB_BASENAME}-libstdcxx-install

unset RPCLIB_BASENAME

# ==============================================================================
# -- Get Recast&Detour and compile it with libc++ ------------------------------
# ==============================================================================

# RECAST_COMMIT="c40188c796f089f89a42e0b939d934178dbcfc5c"
# RECAST_BASENAME=recast-${CXX_TAG}
RECAST_HASH=cdce4e
RECAST_COMMIT=cdce4e1a270fdf1f3942d4485954cc5e136df1df
RECAST_BASENAME=recast-${RECAST_HASH}-${CXX_TAG}

RECAST_INCLUDE=${LIB_HEADER_INCLUDE_PATH}
RECAST_LIBPATH=${CARLA_BUILD_FOLDER}

# if [[ -d "${RECAST_BASENAME}-install" ]] ; then
#   log "${RECAST_BASENAME} already installed."
# else
rm -Rf \
    ${RECAST_BASENAME}-source \
    ${RECAST_BASENAME}-build \
    ${RECAST_BASENAME}-install

log "Retrieving Recast & Detour"

# git clone https://github.com/recastnavigation/recastnavigation.git ${RECAST_BASENAME}-source
git clone https://github.com/carla-simulator/recastnavigation.git ${RECAST_BASENAME}-source

pushd ${RECAST_BASENAME}-source >/dev/null

git reset --hard ${RECAST_COMMIT}

popd >/dev/null

log "Building Recast & Detour with libc++."

mkdir -p ${RECAST_BASENAME}-build

pushd ${RECAST_BASENAME}-build >/dev/null

cmake -G "Unix Makefiles" \
    -DCMAKE_CXX_FLAGS="-std=c++14 -fPIC" \
    -DCMAKE_INSTALL_PREFIX="../${RECAST_BASENAME}-install" \
    -DRECASTNAVIGATION_DEMO=False \
    -DRECASTNAVIGATION_TEST=False \
    ../${RECAST_BASENAME}-source

make -j${LIB_BUILD_CONCURRENCY}

make install

popd >/dev/null

rm -Rf ${RECAST_BASENAME}-source ${RECAST_BASENAME}-build

# move headers inside 'recast' folder
mkdir -p "${PWD}/${RECAST_BASENAME}-install/include/recast"
mv "${PWD}/${RECAST_BASENAME}-install/include/"*h "${PWD}/${RECAST_BASENAME}-install/include/recast/"
cp -r ${RECAST_BASENAME}-install/include/recast ${RECAST_INCLUDE}/recast
cp -r ${RECAST_BASENAME}-install/lib/* ${RECAST_LIBPATH}/ >/dev/null

rm -rf ${RECAST_BASENAME}-install

# fi

unset RECAST_BASENAME

# ==============================================================================
# -- Get and compile libpng 1.6.37 ------------------------------
# ==============================================================================

LIBPNG_VERSION=1.6.37
LIBPNG_REPO=https://sourceforge.net/projects/libpng/files/libpng16/${LIBPNG_VERSION}/libpng-${LIBPNG_VERSION}.tar.xz
LIBPNG_BASENAME=libpng-${LIBPNG_VERSION}
LIBPNG_INSTALL=${LIBPNG_BASENAME}-install

LIBPNG_INCLUDE=${LIB_HEADER_INCLUDE_PATH}
LIBPNG_LIBPATH=${CARLA_BUILD_FOLDER}

log "Retrieving libpng."
wget ${LIBPNG_REPO}

log "Extracting libpng."
tar -xf libpng-${LIBPNG_VERSION}.tar.xz
mv ${LIBPNG_BASENAME} ${LIBPNG_BASENAME}-source

pushd ${LIBPNG_BASENAME}-source >/dev/null

echo "${CARLA_BUILD_FOLDER}/${LIBPNG_INSTALL}"
./configure --prefix=${CARLA_BUILD_FOLDER}/${LIBPNG_INSTALL}
make install

popd >/dev/null

cp -r ${LIBPNG_BASENAME}-install/include/libpng16 ${LIBPNG_INCLUDE}/libpng16
cp -r ${LIBPNG_BASENAME}-install/lib/* ${LIBPNG_LIBPATH} > /dev/null

rm -Rf libpng-${LIBPNG_VERSION}.tar.xz
rm -Rf ${LIBPNG_BASENAME}-source
rm -Rf ${LIBPNG_BASENAME}-install



# ==============================================================================
# -- Generate Version.h --------------------------------------------------------
# ==============================================================================

CARLA_VERSION=$(get_git_repository_version)

log "CARLA version ${CARLA_VERSION}."

VERSION_H_FILE=${CARLA_ROOT_FOLDER}/third_party/LibCarla/source/carla/Version.h
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
set(RECAST_INCLUDE_PATH "${RECAST_INCLUDE}")
set(LIBPNG_INCLUDE_PATH "${LIBPNG_INCLUDE}")
set(RECAST_LIB_PATH "${RECAST_LIBPATH}")
set(LIBPNG_LIB_PATH "${LIBPNG_LIBPATH}")


EOL

# -- Move files ----------------------------------------------------------------

move_if_changed "${CMAKE_CONFIG_FILE}.gen" "${CMAKE_CONFIG_FILE}"

# ==============================================================================
# -- ...clean up ---------------------------------------------------------------
# ==============================================================================
rm -r cmake pkgconfig

popd >/dev/null
rm -rf ${LIBCARLA_BUILD_PATH}

# mv ${CARLA_BUILD_FOLDER}/include/* ${CARLA_ROOT_FOLDER}/include/system/


# ==============================================================================
# -- ...and we are done --------------------------------------------------------
# ==============================================================================

log "Success!"
