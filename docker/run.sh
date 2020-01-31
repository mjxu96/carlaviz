echo "Launching backend"
./carla-display-backend/bin/platform &
sleep 3
echo "backend launched."

echo "Launching frontend"
cd ./carla-display-frontend/
yarn start-live &
echo "frontend launched. Please open your browser"
sleep infinity