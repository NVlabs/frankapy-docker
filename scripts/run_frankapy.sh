#!/bin/bash

set -e
set -u

if [ -z ${FRANKAPY_DIR+x} ]; then
  opts=""
else
  opts="-v $FRANKAPY_DIR:/root/frankapy"
fi

FRANKAPY_DOCKER_DIR=$( cd $( dirname ${BASH_SOURCE[0]} )/../ && pwd )

xhost >& /dev/null && xhost +
docker run \
  -it \
  -e DISPLAY \
  --gpus all \
  --net host \
  --privileged \
  -v $FRANKAPY_DOCKER_DIR:/root/frankapy-docker \
  $opts \
  frankapy \
  /bin/bash
xhost >& /dev/null && xhost -
