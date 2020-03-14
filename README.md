
# CarlaViz
Visualize carla in the web browser.

Now support Carla 0.9.6

<img src="https://github.com/wx9698/carlaviz/raw/master/docs/images/demo.png" style="width: 80%; margin-left: 10%"></img>

## Instructions
### Docker image
Docker image is provided. Run following command to pull this image (including frontend and backend).

```bash
# pull the image
docker pull mjxu96/carlaviz:0.9.6
```

### How to run it?
Here is a simple [example](https://github.com/wx9698/carlaviz/tree/master/examples) to run it. This example also includes the [Python API](https://github.com/wx9698/carlaviz/blob/master/examples/carla_painter.py) to draw extra lines, points and texts in the web browser.

### Build from sources
Although the container image is out-of-box to use, you are also welcome to build on your own!

Refer to this [page](https://github.com/wx9698/carlaviz/blob/master/docs/build.md) for building instructions.


## Used Libraries
1. [uber streetscape.gl](https://github.com/uber/streetscape.gl) as frontend
2. [carla](http://carla.org/)
3. [mjxu96 xviz](https://github.com/wx9698/xviz)
4. [boost](https://www.boost.org/)