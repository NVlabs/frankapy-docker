#!/bin/bash

tmux new-session -d
tmux split-window -h
tmux send-keys -t 0 "roscore" Enter
tmux attach-session
