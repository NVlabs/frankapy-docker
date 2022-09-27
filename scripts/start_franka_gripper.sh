#!/bin/bash

roslaunch franka_ros_interface franka_gripper.launch \
  robot_num:=1 \
  robot_ip:="172.16.0.2"
