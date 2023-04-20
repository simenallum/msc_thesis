#!/bin/bash
tmux send-keys "cd ~/" Enter
tmux send-keys ". /opt/ros/noetic/setup.bash" Enter
tmux send-keys ". ~/catkin_ws/devel/setup.bash" Enter
tmux send-keys "clear" Enter