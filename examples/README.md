
## CarlaViz example

This example spawns one ego vehicle with two other vehicles and attaches a camera and a lidar to the ego vehicle.

### Prerequisite:
1. python3
2. [carla](https://pypi.org/project/carla/)

### Useage
```bash
# 1. launch carla simulator
cd CARLA_SIMULATOR_PATH
./CarlaUE4.sh

# 2. run the docker image
# Linux
docker run -it --network="host" mjxu96/carlaviz:0.9.15 \
  --simulator_host localhost \
  --simulator_port 2000

# Windows/MacOS
docker run -it -p 8080-8081:8080-8081 mjxu96/carlaviz:0.9.15 \
  --simulator_host host.docker.internal \
  --simulator_port 2000

# 3. run this script
python3 example.py

# 4. open your browser and go to localhost:8080
```

### [NOT IMPLEMENTED YET] ~~Python API~~
~~A simple [python class (CarlaPainter)](./carla_painter.py) is provided to draw polylines, points and texts in the web browser.~~
