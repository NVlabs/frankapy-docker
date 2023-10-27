#!/bin/bash

FRANKAPY_DOCKER_DIR=$( cd $( dirname ${BASH_SOURCE[0]} )/.. && pwd )

xhost +local:root
docker run \
  -it \
  -e DISPLAY \
  -e CONSOLE \
  -e ROS_MASTER_URI \
  --net host \
  --privileged \
  -v $FRANKAPY_DOCKER_DIR:/root/frankapy-docker \
  -v $FRANKAPY_DOCKER_DIR/config/terminator_config:/root/.config/terminator/config \
  franka-interface \
  bash -ci ' \
    if [ ! -z ${DISPLAY+x} ] && ( [ -z ${CONSOLE+x} ] || [ $CONSOLE == terminator ] ); then \
      ./frankapy-docker/scripts/start_control_pc_terminator.sh; \
    elif [ -z ${CONSOLE+x} ] || [ $CONSOLE == tmux ]; then \
      ./frankapy-docker/scripts/start_control_pc_tmux.sh; \
    fi; \
    bash \
  '
xhost -local:root
