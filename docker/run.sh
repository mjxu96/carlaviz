cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

echo -e "HOST=${CARLAVIZ_HOST}" >> /home/carla/.env
echo -e "PORT=${CARLAVIZ_PORT}" >> /home/carla/.env

echo "Make sure you have launched the carla server."
echo "Launching backend."
./backend/bin/backend ${CARLA_SERVER_IP} ${CARLA_SERVER_PORT} ${CARLAVIZ_PORT} &
sleep 5

echo "Backend launched."
echo "Launching frontend"

sleep 2
cd ./frontend/
yarn start | tee frontend.log &
while ! grep "Compiled successfully" .log &> /dev/null; do sleep 1; done
echo "Frontend launched. Please open your browser"
sleep infinity
