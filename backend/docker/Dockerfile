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

RUN apt install -y git build-essential gcc-7 cmake libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl wget unzip autoconf libtool rsync && \
  cd /home/carla/ && \
  git clone https://github.com/mellocolate/carla-display-backend.git && \
  cd carla-display-backend && \
  ./setup/setup.sh && \
  mkdir build && \
  cd build && \
  cmake ../ && \
  make platform -j12


# frontend part

# install nodejs 10.x
RUN curl -sL https://deb.nodesource.com/setup_10.x |  bash - && \
  apt-get install -y nodejs

# install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - && \
  echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list && \
  apt-get update && apt-get install -y yarn

RUN apt install -y make gcc g++ && \
  cd /home/carla/ && \
  git clone https://github.com/mellocolate/carla-display-frontend.git && \
  cd carla-display-frontend && \
  yarn

EXPOSE 8080-8081

USER carla
ENV CARLA_DISPLAY_HOST_IP localhost

COPY run.sh /home/carla/
WORKDIR /home/carla

CMD ["./run.sh"]

ENTRYPOINT ["/bin/bash", "-c"]
