FROM ubuntu:18.04

RUN useradd -ms /bin/bash carla
USER carla

RUN mkdir /home/carla/protoc

USER root

RUN apt update && \
  apt install -y wget autoconf automake libtool curl make g++ unzip

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
RUN apt update && \
  apt install -y git build-essential gcc-7 cmake libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl wget unzip autoconf libtool rsync

# install nodejs 12.x
RUN curl -sL https://deb.nodesource.com/setup_12.x |  bash - && \
  apt-get install -y nodejs

# install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - && \
  echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list && \
  apt-get update && apt-get install -y yarn

RUN cd /home/carla && \
  git clone --recurse-submodules https://github.com/wx9698/carlaviz.git && cd carlaviz && \
  echo `git describe --tags` && \
  bash ./setup/setup.sh

EXPOSE 8080-8081
EXPOSE 8089

USER carla
ENV CARLAVIZ_HOST_IP localhost
ENV CARLAVIZ_HOST_PORT 8081
ENV CARLA_SERVER_IP localhost
ENV CARLA_SERVER_PORT 2000

WORKDIR /home/carla/carlaviz

CMD ["./docker/run.sh"]

ENTRYPOINT ["/bin/bash", "-c"]
