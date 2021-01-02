cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

echo -e "CARLAVIZ_HOST=${CARLAVIZ_HOST}" >> ~/.env
echo -e "CARLAVIZ_PORT=${CARLAVIZ_PORT}" >> ~/.env

echo "Make sure you have launched the carla server."
echo "Launching backend."
./backend/bin/backend ${CARLA_SERVER_IP} ${CARLA_SERVER_PORT} ${CARLAVIZ_PORT} | tee backend.log &
sleep 5

echo "Backend launched."
echo "Launching frontend"

sleep 2
cd frontend/
yarn start | tee ../frontend.log &
while ! grep "Compiled successfully" ../frontend.log &> /dev/null; do sleep 1; done
echo "Frontend launched. Please open your browser"
sleep infinity
