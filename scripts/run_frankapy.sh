#!/bin/bash

if [ -z ${FRANKAPY_DIR+x} ]; then
  opts=""
else
  opts="-v $FRANKAPY_DIR:/root/frankapy"
fi

FRANKAPY_DOCKER_DIR=$( cd $( dirname ${BASH_SOURCE[0]} )/.. && pwd )

xhost +local:root
docker run \
  -it \
  -e DISPLAY \
  -e CONSOLE \
  --gpus all \
  --net host \
  --privileged \
  -v $FRANKAPY_DOCKER_DIR:/root/frankapy-docker \
  -v $FRANKAPY_DOCKER_DIR/config/terminator_config:/root/.config/terminator/config \
  $opts \
  frankapy \
  bash -ci ' \
    if [ ! -z ${DISPLAY+x} ] && ( [ -z ${CONSOLE+x} ] || [ $CONSOLE == terminator ] ); then \
      ./frankapy-docker/scripts/start_frankapy_pc_terminator.sh; \
    elif [ -z ${CONSOLE+x} ] || [ $CONSOLE == tmux ]; then \
      ./frankapy-docker/scripts/start_frankapy_pc_tmux.sh; \
    fi; \
    bash \
  '
xhost -local:root
