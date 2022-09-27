#!/bin/bash

set -e
set -u

FRANKAPY_DOCKER_DIR=$( cd $( dirname ${BASH_SOURCE[0]} )/../ && pwd )

xhost >& /dev/null && xhost +
docker run \
  -it \
  -e DISPLAY \
  --net host \
  --privileged \
  -v $FRANKAPY_DOCKER_DIR:/root/frankapy-docker \
  franka-interface \
  /bin/bash
xhost >& /dev/null && xhost -
