# Frontend build stage
FROM node:18-alpine AS frontend

COPY ./frontend /home/carlaviz/frontend

RUN apk --no-cache add git

WORKDIR /home/carlaviz/frontend
RUN yarn
RUN yarn build

# Backend build stage
FROM ubuntu:22.04 AS backend

RUN apt update
RUN apt install -y make g++-11 pip cmake
RUN pip3 install conan==1.55.0

# Add conan registry for xviz
RUN conan remote add gitlab https://gitlab.com/api/v4/projects/44861904/packages/conan
COPY ./misc/cicd/conan/gcc11 /home/carlaviz/profiles/
COPY ./backend /home/carlaviz/backend

# build carlaviz backend
WORKDIR /home/carlaviz/backend/build
RUN conan install -pr /home/carlaviz/profiles/gcc11 --build=missing -s build_type=Release ..
RUN conan build ..

# Release stage
FROM nginx:1.25.1

# frontend
COPY --from=frontend /home/carlaviz/frontend/dist/ /var/www/carlaviz/
COPY --from=frontend /home/carlaviz/frontend/index.html /var/www/carlaviz/index.html
COPY ./misc/docker/carlaviz /etc/nginx/conf.d/default.conf

# backend
COPY --from=backend /home/carlaviz/backend/build/Release/src/backend /home/carlaviz/backend/bin/backend

COPY ./misc/docker/run.sh /home/carlaviz/docker/run.sh

EXPOSE 8080-8081
EXPOSE 8089

WORKDIR /home/carlaviz

ENTRYPOINT ["/bin/bash", "./docker/run.sh"]
