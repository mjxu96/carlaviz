
# backend part
BACKEND_DIR=$(cd "$(dirname "$0")"; cd ../backend; pwd)
pushd ${BACKEND_DIR} > /dev/null
bash ./setup/setup.sh
mkdir build && cd build
cmake ../
make backend -j12
popd > /dev/null
