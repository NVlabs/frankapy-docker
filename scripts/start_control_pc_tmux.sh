#!/bin/bash

tmux new-session -d
tmux split-window -h
tmux split-window -v -t 1
tmux send-keys -t 0 "$HOME/frankapy-docker/scripts/start_franka_interface.sh" Enter
tmux send-keys -t 1 "sleep 2 && $HOME/frankapy-docker/scripts/start_franka_ros_interface.sh" Enter
tmux send-keys -t 2 "sleep 2 && $HOME/frankapy-docker/scripts/start_franka_gripper.sh" Enter
tmux attach-session
