#!/bin/bash

FRANKAPY_DOCKER_DIR=$( cd $( dirname ${BASH_SOURCE[0]} )/.. && pwd )

xhost +local:root
docker run \
  -it \
  -e DISPLAY \
  --net host \
  --privileged \
  -v $FRANKAPY_DOCKER_DIR:/root/frankapy-docker \
  franka-interface \
  /bin/bash
xhost -local:root
