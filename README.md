# Carla Display on Browser - Backend

Please refer this [repo](https://github.com/mellocolate/carla-display) for building instructions.

# install nodejs 10.x
```bash
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
```

# install yarn
```bash
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update && sudo apt-get install -y yarn
```

# install make gcc g++
```bash
sudo apt install -y make gcc g++ 
```

# download carla release version
```bash
mkdir carla-simulator
pushd carla-simulator > /dev/null
wget http://carla-assets-internal.s3.amazonaws.com/Releases/Linux/CARLA_0.9.6.tar.gz
tar xvzf CARLA_0.9.6.tar.gz
popd > /dev/null
```

# download nvidia driver
```bash
mkdir nvidia-driver
pushd nvidia-driver > /dev/null
wget http://us.download.nvidia.com/tesla/418.67/NVIDIA-Linux-x86_64-418.67.run
sudo bash ./NVIDIA-Linux-x86_64-418.67.run
popd > /dev/null
```
