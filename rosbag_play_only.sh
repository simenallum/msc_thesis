#!/bin/bash

usage="Usage: . rosbag_play_only.sh <path_to_bag> <env>"

if [ $# -ne 2 ]
  then
    echo $usage
    exit
fi

path=$1
ENV=$2

EVAL_SEG_TOPICS="/anafi/image \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/pose \
        /anafi/rpy \
        /anafi/ned_pos_from_gnss \
        /anafi/ned_frame_gnss_origin \
        /anafi/optical_flow_velocities \
        /anafi/polled_body_velocities"

EVAL_PIX2GEO_TOPICS="/anafi/image \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/pose \
        /anafi/rpy \
        /anafi/ned_pos_from_gnss \
        /qualisys/AT0/pose \
        /qualisys/Anafi/pose"

EVAL_PT_TOPICS="/anafi/image \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/pose \
        /anafi/rpy \
        /anafi/ned_pos_from_gnss \
        /qualisys/Anafi/pose \
        /anafi/polled_body_velocities \
        /qualisys/Platform/pose"

if [[ $ENV == "outside" ]]; then
    echo "Rosbag play outside topics"
    rosbag play $path --clock --topics $EVAL_SEG_TOPICS
elif [[ $ENV == "eval_pix2geo" ]]; then
    echo "Rosbag play eval pix2geo topics"
    rosbag play $path --clock --topics $EVAL_PIX2GEO_TOPICS
elif [[ $ENV == "eval_pt" ]]; then
    echo "Rosbag play eval PT topics"
    rosbag play $path --clock --topics $EVAL_PT_TOPICS
fi