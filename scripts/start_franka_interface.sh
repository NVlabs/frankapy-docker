#!/bin/bash

$HOME/franka-interface/build/franka_interface \
  --robot_ip "172.16.0.2" \
  --with_gripper 0 \
  --log 0 \
  --stop_on_error 0
