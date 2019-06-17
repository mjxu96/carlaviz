# carla c++ client for 095 release version

This repo is used to generate carla c++ client with least source files. 

It will also try to use GNU build tools (like g++ instead of clang++) to build this client.

usage:
```
$ git clone https://github.com/wx9698/carla-client-cpp.git
$ cd carla-client-cpp
$ sudo ./install-carla-tool.sh             # install necessary carla build tools
$ ./setup/setup.sh
$ cd src/example
$ make build
```
