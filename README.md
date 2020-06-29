
# CarlaViz [![Build Status](https://travis-ci.com/wx9698/carlaviz.svg?branch=master)](https://travis-ci.com/wx9698/carlaviz)
Visualize carla in the web browser.

Support Carla 0.9.6, 0.9.7, 0.9.8 and 0.9.9 with source files or docker on Linux.

Support Carla 0.9.9 with docker on Windows.

Also this fork can use latest CARLA feature of multi-streams

<img src="https://github.com/carla-simulator/carlaviz/raw/master/docs/images/demo1.png"></img>

## Instructions
### Docker image
Docker image is provided. Run following command to pull this image (including frontend and backend).

```bash
# pull the image based on your carla version (single-stream)
docker pull mjxu96/carlaviz:0.9.6
docker pull mjxu96/carlaviz:0.9.7
docker pull mjxu96/carlaviz:0.9.8
docker pull mjxu96/carlaviz:0.9.9
# pull the image to use latest CARLA multi-stream feature
docker pull carlasim/carlaviz:latest
```

### How to run it?
Here is a simple [example](https://github.com/carla-simulator/carlaviz/tree/master/examples) to run it. This example also includes the [Python API](https://github.com/carla-simulator/carlaviz/blob/master/examples/carla_painter.py) to draw extra lines, points and texts in the web browser.

### Build from source
Although the container image is out-of-box to use, you are also welcome to build on your own!

Refer to this [page](https://github.com/carla-simulator/carlaviz/blob/master/docs/build.md) for building instructions.

## Author
[Minjun Xu](https://github.com/wx9698)   mjxu96@gmail.com

## Used Libraries
1. [uber streetscape.gl](https://github.com/uber/streetscape.gl) as frontend
2. [carla](http://carla.org/)
3. [mjxu96 xviz](https://github.com/wx9698/xviz)
4. [boost](https://www.boost.org/)
