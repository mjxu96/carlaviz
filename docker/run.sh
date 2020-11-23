cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

echo -e "CARLAVIZ_HOST_IP=${CARLAVIZ_HOST_IP}" >> /home/carla/.env
echo -e "CARLAVIZ_HOST_PORT=${CARLAVIZ_HOST_PORT}" >> /home/carla/.env

echo "Make sure you have launched the carla server."
echo "Launching backend."
./backend/bin/backend ${CARLA_SERVER_IP} ${CARLA_SERVER_PORT} &
sleep 5

echo "Backend launched."
echo "Launching frontend"

sleep 2
cd ./frontend/
yarn start &
sleep 10
echo "Frontend launched. Please open your browser"
sleep infinity
