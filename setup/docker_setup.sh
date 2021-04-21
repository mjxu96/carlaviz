
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
mkdir -p /var/www/carlaviz
mv dist/* index.html /var/www/carlaviz/
mv ../setup/carlaviz /etc/nginx/sites-enabled
rm -rf *
popd
