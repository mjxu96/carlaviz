
# CarlaViz [![Build Status](https://travis-ci.com/wx9698/carlaviz.svg?branch=master)](https://travis-ci.com/wx9698/carlaviz)
Visualize carla in the web browser.

<img src="https://github.com/wx9698/carlaviz/raw/master/docs/images/demo1.png"></img>

## Version Supported
| Platform | Supported Carla Version | Docker/Source |
| ------------- | --------------------- | --- |
| Ubuntu | 0.9.6, 0.9.7, 0.9.8, 0.9.9.2, 0.9.9.4, 0.9.10, 0.9.10.1 | Docker image and source |
| Windows | 0.9.9.2, 0.9.10, 0.9.10.1 | Only docker image |

## Instructions
### Docker image
Docker image is provided. Run following command to pull this image (including frontend and backend).

```bash
# pull the image based on your carla version
docker pull mjxu96/carlaviz:0.9.6
docker pull mjxu96/carlaviz:0.9.7
docker pull mjxu96/carlaviz:0.9.8
docker pull mjxu96/carlaviz:0.9.9.2
docker pull mjxu96/carlaviz:0.9.9.4
docker pull mjxu96/carlaviz:0.9.10
docker pull mjxu96/carlaviz:0.9.10.1
docker pull mjxu96/carlaviz:latest
```

### How to run it?
Here is a simple [example](https://github.com/wx9698/carlaviz/tree/master/examples) to run it. This example also includes the [Python API](https://github.com/wx9698/carlaviz/blob/master/examples/carla_painter.py) to draw extra lines, points and texts in the web browser.

### Build from source
Although the container image is out-of-box to use, you are also welcome to build on your own!

Refer to this [page](https://github.com/wx9698/carlaviz/blob/master/docs/build.md) for building instructions.

## Author
[Minjun Xu](https://github.com/wx9698)   mjxu96@gmail.com

## Used Libraries
1. [uber streetscape.gl](https://github.com/uber/streetscape.gl) as frontend
2. [carla](http://carla.org/)
3. [mjxu96 xviz](https://github.com/wx9698/xviz)
4. [boost](https://www.boost.org/)
