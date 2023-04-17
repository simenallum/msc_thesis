#!/bin/bash

usage="Usage: $(basename "$0") <path_to_bag> <env>"

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
        /tf \
        /tf_static \
        /anafi/ned_pos_from_gnss"

if [[ $ENV == "outside" ]]; then
    echo "Rosbag play outside topics"
    rosbag play $path --topics $EVAL_SEG_TOPICS
fi