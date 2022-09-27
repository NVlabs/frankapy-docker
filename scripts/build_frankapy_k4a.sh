#!/bin/bash

docker build \
  -f $( dirname ${BASH_SOURCE[0]} )/../dockerfiles/frankapy_k4a.dockerfile \
  --no-cache \
  -t frankapy_k4a \
  .
