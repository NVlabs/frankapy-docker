#!/bin/bash

tmux new-session -d -x $(tput cols) -y $(tput lines)
tmux split-window -h
tmux send-keys -t 0 "roscore" Enter
tmux attach-session
