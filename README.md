# carla c++ client for 095 release version

This repo is used to generate carla c++ client with least source files. 

It will also try to use GNU build tools (like g++ instead of clang++) to build this client. For fast build, this repo also retain the way to build with clang-7.

usage:
```
$ git clone https://github.com/wx9698/carla-client-cpp.git
$ cd carla-client-cpp
$ sudo ./setup/install-carla-tool.sh             # install necessary carla build tools
$ ./setup/setup.sh                               
# you can use command ./setup/setup.sh clang to indicate script to build with clang
$ mkdir build && cd build
$ cmake ../
# you can also use command cmake -DUSE_CLANG=ON ../ to indicate cmake to set default compiler to clang
$ make main                                      # it will generate the binary program at REPO_ROOT_FOLDER/bin
$ cd && ./bin/main
```
