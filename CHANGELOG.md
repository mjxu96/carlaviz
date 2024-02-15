# Change Logs

## 0.9.15
* Support Carla 0.9.15
* Update Github CI/CD actions version

## 0.9.14.1
* Support Carla 0.9.14.1
* Completely rewrite the backend part
    * Use [Conan](https://docs.conan.io/1/index.html) to manage dependency 
    * Use latest [xviz](https://github.com/mjxu96/xviz) library
    * Update the backend code struture to potentially support more simulators and frontend connections
* Support changing map without exiting the backend
* Render static objects (e.g. buildings, traffic lights) in the browser
* Move more static parts to the metadata to reduce the real time traffic
* Rely on Github Actions for CI/CD
* Start writing CHANGELOG :)
