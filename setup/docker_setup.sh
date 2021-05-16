
set -e

# remove unnecessary files

# backend part cleanup
BACKEND_DIR=$(cd "$(dirname "$0")"; cd ../backend; pwd)
pushd ${BACKEND_DIR} > /dev/null
rm -rf build lib include third_party src
popd

FRONTEND_DIR=$(cd "$(dirname "$0")"; cd ../frontend; pwd)
pushd ${FRONTEND_DIR} > /dev/null
# build bundle js files
yarn build
popd
