#!/bin/bash

run_perception_master=true
run_platform_tracker=false
run_segmentation=true
run_search=false
start_roscore=false


tmux new-session -d
tmux set -g mouse on


if [ "$start_roscore" = true ]; then
    . source_noetic.sh
    tmux send-keys "roscore" Enter
fi

if [ "$run_perception_master" = true ]; then
    tmux new-window -n perception_master

    . open_4_tmux_windows.sh

    tmux select-pane -t 0
    tmux send-keys "roslaunch perception_master perception_master.launch --wait" Enter


    tmux select-pane -t 1
    tmux send-keys "roslaunch pix2geo pix2geo.launch --wait" Enter

    tmux select-pane -t 2
    tmux send-keys "roslaunch transform_publisher transform_publisher.launch --wait" Enter

    tmux select-pane -t 3
    tmux send-keys "bash ~/catkin_ws/src/msc_thesis/init_camera.sh" Enter
    tmux send-keys "nvidia-smi -l"

    # tmux split-window -v
    # . source_noetic.sh
    # tmux send-keys "rostopic echo /perception/detected_person" Enter

fi

if [ "$run_platform_tracker" = true ]; then
    tmux new-window -n platform_tracker

    . open_4_tmux_windows.sh

    tmux select-pane -t 0
    tmux send-keys "roslaunch perception_ekf perception_ekf_node.launch --wait" Enter

    tmux select-pane -t 1
    tmux send-keys "roslaunch dnncv dnnCV.launch --wait" Enter

    tmux split-window -v
    . source_noetic.sh

    tmux send-keys "roslaunch yolov8_ros platform_node.launch --wait" Enter

    tmux select-pane -t 3
    tmux send-keys "roslaunch apriltags_pose_estimator apriltags_pose_estimator.launch --wait" Enter
    
fi

if [ "$run_segmentation" = true ]; then
    tmux new-window -n segmentation

    . open_4_tmux_windows.sh

    tmux select-pane -t 0
    tmux send-keys "roslaunch water_segmentation segmentation_master.launch --wait" Enter

    tmux select-pane -t 2
    tmux send-keys "roslaunch water_segmentation map_segmentation.launch --wait" Enter

    tmux select-pane -t 3
    tmux send-keys "roslaunch water_segmentation dl_segmentation.launch --wait" Enter
fi

if [ "$run_search" = true ]; then
    tmux new-window -n search

    . open_4_tmux_windows.sh

    tmux select-pane -t 0
    tmux send-keys "roslaunch sort_tracker sort_tracker.launch --wait" Enter

    tmux select-pane -t 2
    tmux send-keys "roslaunch yolov8_ros search_node.launch --wait" Enter
    # tmux send-keys "roslaunch test_tools detector_simulator.launch --wait" Enter
fi

tmux select-window -t 1
tmux select-pane -t 0
tmux attach-session