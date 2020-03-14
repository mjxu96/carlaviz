cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    sleep 2
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

echo -e "CARLAVIZ_HOST_IP=${CARLAVIZ_HOST_IP}" >> /home/carla/.env

echo "Make sure you have launched the carla server."
echo "Launching backend."
./carlaviz/backend/bin/backend &
sleep 5

echo "Backend launched."
echo "Launching frontend"

sleep 2
cd ./carlaviz/frontend/
yarn start-live &
sleep 10
echo "Frontend launched. Please open your browser"
sleep infinity
