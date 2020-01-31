# build docker
docker build --rm  -t mellocolate/backend:0.1 .

# run docker
docker run -it -p 2000-2002:2000-2002 --gpus all mellocolate/backend:0.1 /bin/sh