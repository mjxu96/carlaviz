FROM ubuntu:18.04

RUN useradd -ms /bin/bash carla
USER carla

RUN mkdir /home/carla/carla-simulator
RUN mkdir /home/carla/protoc

USER root


RUN apt update && \
  apt install -y wget autoconf automake libtool curl make g++ unzip

RUN cd /home/carla/carla-simulator && \
  wget http://carla-assets-internal.s3.amazonaws.com/Releases/Linux/CARLA_0.9.6.tar.gz && \
  tar xvzf CARLA_0.9.6.tar.gz && \
  rm CARLA_0.9.6.tar.gz


RUN cd /home/carla/protoc/ && \
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-cpp-3.11.2.tar.gz && \
  tar xvzf protobuf-cpp-3.11.2.tar.gz && \
  cd protobuf-3.11.2 && \
  ./configure && \
  make -j12 && \
  make install && \
  ldconfig

# non-interactive setting for tzdata
ENV DEBIAN_FRONTEND=noninteractive

RUN apt install -y git build-essential gcc-7 cmake libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl wget unzip autoconf libtool rsync && \
  cd /home/carla/ && \
  git clone https://github.com/mellocolate/carla-display-backend.git && \
  cd carla-display-backend && \
  ./setup/setup.sh && \
  git checkout test_new_xviz_lib && \
  mkdir build && \
  cd build && \
  cmake ../ && \
  make platform -j12

EXPOSE 2000-2002
EXPOSE 8080-8081


# CMD /bin/sh -c "/carla-simulator/Carla"

# CMD ["ls", "/home/carla/carla-display-backend/"]

ENTRYPOINT ["/bin/bash", "-c"]