
## CarlaViz example

This example spawns one ego vehicle with two other vehicles and attaches a camera and a lidar to the ego vehicle. It also draws the trajectory and velocity of the ego vehicle.

### Prerequisite:
1. python3
2. websocket_client (pip3 install websocket_client)

### Useage
```bash
# 1. launch carla simulator
cd CARLA_SIMULATOR_PATH
./CarlaUE4.sh

# 2. launch container
#    if you run this command in a remote machine, replace localhost 
#    with the ip address of the machine where you run this command, 
#    otherwise, keep it as localhost
docker run -it --network="host" -e CARLAVIZ_HOST_IP=localhost mjxu96/carlaviz:0.9.6

# 3. run this script
python3 example.py
```

### Python API
A simple [python class (CarlaPainter)]() is provided to draw polylines, points and texts in the web browser.