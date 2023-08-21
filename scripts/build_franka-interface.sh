#!/bin/bash

docker build \
  -f $( dirname ${BASH_SOURCE[0]} )/../dockerfiles/franka-interface.dockerfile \
  --no-cache \
  -t franka-interface:rvt \
  .
