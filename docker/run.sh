
echo "Launching carla-simulator"
./carla-simulator/CarlaUE4.sh &
sleep 3
echo "carla-simulator launched."

echo "Launching backend"
./carla-display-backend/bin/platform &
sleep 3
echo "backend launched."
sleep infinity

echo "Launching frontend"
cd ./carla-display-frontend/
yarn start-live &
sleep 10
echo "frontend launched. Please open your browser"
sleep infinity