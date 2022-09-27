#!/bin/bash

docker build \
  -f $( dirname ${BASH_SOURCE[0]} )/../dockerfiles/frankapy.dockerfile \
  --no-cache \
  -t frankapy \
  .
