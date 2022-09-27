#!/bin/bash

mkdir -p $HOME/.config/terminator

rsync $HOME/frankapy-docker/scripts/terminator_config $HOME/.config/terminator/config

terminator -l start_frankapy_pc
