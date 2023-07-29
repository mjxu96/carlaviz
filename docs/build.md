# Building Instructions

Before start, the [docker image](https://github.com/mjxu96/carlaviz#docker-image) is also provided. It is easier to use and has fewer prerequisites than building from source files.

## Prerequisites
#### Backend part
1. C++20 Compiler (gcc11, clang14 or msvc17) and CMake
2. OS: windows or ubuntu 22.04 (otherwise you need to get glibc >= 2.32)
3. [Conan](https://docs.conan.io/1/index.html) >= 1.55.0, profiles can be found [here](../misc/cicd/conan)
#### Frontend part
1. nodejs 18.x
2. yarn

## Step by step
```bash
# 1. clone this repo with submodules
git clone https://github.com/mjxu96/carlaviz.git

# 2. build backend
cd backend
mkdir build && cd build
# add extra conan registry to retrieve xviz and libcarla
conan remote add gitlab https://gitlab.com/api/v4/projects/44861904/packages/conan
conan install -pr gcc11 -s build_type=Debug --build=missing ..
conan build ..

# 3. build frontend
cd frontend
yarn
```

And you have successfully built it!

## How to use it?
```bash
# 1. run carla simulator server
cd CARLA_SIMULATOR_PATH
./CarlaUE4.sh

# 2. in another terminal, run backend
./backend/build/Debug/src/backend \
  --simulator_host localhost \
  --simulator_port 2000
# More usage and parameters can check below command
# ./backend/build/Debug/src/backend --help

# 3. in another terminal, run frontend
cd frontend
yarn start

# 4. in another terminal, run the python script
cd examples
python3 example.py

# 5. go to the browser and type localhost:8080
```
