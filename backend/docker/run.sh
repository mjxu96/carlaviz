cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    sleep 2
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

echo -e "CARLA_DISPLAY_HOST_IP=${CARLA_DISPLAY_HOST_IP}" >> /home/carla/.env

echo "Launching backend"
./carla-display-backend/bin/platform &
sleep 3
echo "backend launched."

echo "Launching frontend"
cd ./carla-display-frontend/
yarn start-live &
sleep 10
echo "frontend launched. Please open your browser"
sleep infinity
