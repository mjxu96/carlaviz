# Building Instructions

Before start, the [docker image](https://github.com/wx9698/carlaviz#docker-image) is also provided. It is easier to use and has fewer prerequisites than building from source files.

## Prerequisites
#### Backend part
1. Compiler: g++ (>=7) or clang++ (>=7)
2. OS: ubuntu 18.04 (only tested on this os)
3. [Protobuf C++](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md): (>=3.11.0 <=3.11.4)

#### Frontend part
1. nodejs 12.x
2. yarn

Here is the script to download these frontend tools.
```bash
# install nodejs 12.x
sudo apt-get update
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt-get install -y nodejs

# install yarn
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update && sudo apt-get install -y yarn
```

## Step by step
```bash
# 1.1 clone this repo with submodules
git clone --recurse-submodules https://github.com/wx9698/carlaviz.git
# 1.2 (optional) checkout to your carla version
git checkout 0.9.10

# 2. install packages
sudo apt-get update
sudo apt-get install build-essential gcc-7 cmake libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl wget unzip autoconf libtool

# 3. run setup script from the repo's root
cd carlaviz
./setup/setup.sh
```

And you have successfully built it!

## How to use it?
```bash
# 1. run carla simulator server
cd CARLA_SIMULATOR_PATH
./CarlaUE4.sh

# 2. in another terminal, run backend
./backend/bin/backend

# 3. in another terminal, run frontend
cd frontend
yarn start

# 4. in another terminal, run the python script
cd examples
python3 example.py

# 5. go to the browser and type localhost:8080
```