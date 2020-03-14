# exit when error occurs
set -e

# backend part
BACKEND_DIR=$(cd "$(dirname "$0")"; cd ../backend; pwd)
pushd ${BACKEND_DIR} > /dev/null
bash ./setup/setup.sh
mkdir build && cd build
cmake ../
make backend -j8
popd > /dev/null

# frontend part
FRONTEND_DIR=$(cd "$(dirname "$0")"; cd ../frontend; pwd)
pushd ${FRONTEND_DIR} > /dev/null
yarn
popd > /dev/null