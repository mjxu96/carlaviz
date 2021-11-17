# Frontend build stage
FROM node:12-alpine AS frontend

COPY ./frontend /home/carla/carlaviz/frontend

RUN apk --no-cache add git

WORKDIR /home/carla/carlaviz/frontend
RUN yarn && yarn build

# Backend build stage
FROM ubuntu:18.04 AS backend

COPY ./backend /home/carla/carlaviz/backend

RUN apt update && \
    apt install -y wget autoconf automake libtool curl make g++ unzip

# install protobuf for xviz
WORKDIR /home/carla/protoc
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-cpp-3.11.2.tar.gz && \
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

# build carlaviz backend
WORKDIR /home/carla/carlaviz/backend
RUN bash ./setup/setup.sh
WORKDIR /home/carla/carlaviz/backend/build
RUN cmake ../ && make backend -j`nproc --all`

# Release stage
FROM nginx:1.20

# frontend
COPY --from=frontend /home/carla/carlaviz/frontend/dist/ /var/www/carlaviz/
COPY --from=frontend /home/carla/carlaviz/frontend/index.html /var/www/carlaviz/index.html
COPY ./docker/carlaviz /etc/nginx/conf.d/default.conf

# backend
COPY --from=backend /home/carla/carlaviz/backend/bin/backend /home/carla/carlaviz/backend/bin/backend
COPY --from=backend /home/carla/protoc/protobuf-3.11.2/src/.libs/libprotobuf.so.22 /lib/x86_64-linux-gnu/libprotobuf.so.22
COPY --from=backend /home/carla/carlaviz/backend/lib/libboost_filesystem.so.1.72.0 /lib/x86_64-linux-gnu/libboost_filesystem.so.1.72.0

COPY ./docker/run.sh /home/carla/carlaviz/docker/run.sh

EXPOSE 8080-8081
EXPOSE 8089

ENV CARLAVIZ_BACKEND_HOST localhost
ENV CARLAVIZ_BACKEND_PORT 8081
ENV CARLA_SERVER_HOST localhost
ENV CARLA_SERVER_PORT 2000

WORKDIR /home/carla/carlaviz

ENTRYPOINT ["/bin/bash", "./docker/run.sh"]

