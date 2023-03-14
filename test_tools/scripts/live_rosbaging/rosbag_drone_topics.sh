#!/bin/bash

usage="Usage: $(basename "$0") <dataset_name> <test-name> <env (sim/lab/real)>"

if [ $# -ne 3 ]
  then
    echo $usage
    exit
fi

DATASET_NAME=$1
TEST_NAME=$2
ENV=$3
SCRIPT_DIR=$(dirname "$(realpath $0)")

OUTPUT_DIR=$SCRIPT_DIR/rosbags/$ENV/$DATASET_NAME/$TEST_NAME
mkdir -p $OUTPUT_DIR

if [ -e $OUTPUT_DIR/*.bag ]
then
    OLD_DIR=$OUTPUT_DIR/old
    echo "Moving old bagfile into "$OLD_DIR""
    mkdir -p $OLD_DIR
    mv $OUTPUT_DIR/*.bag $OLD_DIR
fi

TIME=$(date +%Y-%m-%d-%H-%M-%S)

EVAL_PIX2GEO="/anafi/image \
              /anafi/attitude \
              /anafi/gnss_location \
              /anafi/height \
              /anafi/optical_flow_velocities \
              /anafi/polled_body_velocities \
              /anafi/pose \
              /anafi/odometry \
              /anafi/rpy \
              /tf \
              /anafi/ned_pos_from_gnss \
              /qualisys/Anafi/odom \
              /qualisys/Anafi/pose \
              /qualisys/Anafi/velocity \
              /qualisys/AT0/odom \
              /qualisys/AT0/pose \
              /qualisys/AT0/velocity \
              /search/tracks \
              /yolo/search/boxes"

ANAFI_PERCEPTION_RELEVANT_TOPICS="/anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /darknet_ros/bounding_boxes \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/pose \
        /anafi/odometry \
        /anafi/rpy \
        /anafi/polled_body_velocities \
        /anafi/rpy \
        /tf \
        /tf_static \
        /estimate/aprilTags/pose \
        /estimate/aprilTags/pose/camera_frame \
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/aprilTags/num_tags_detected \
        /anafi/ned_pose_from_gnss \
        /anafi/gnss_ned_in_body_frame \
        /anafi/gnss_ned_in_body_frame/1hz \
        /estimate/ekf/velocity"

OUTSIDE="/anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /darknet_ros/bounding_boxes \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/pose \
        /anafi/odometry \
        /anafi/rpy \
        /anafi/polled_body_velocities \
        /anafi/rpy \
        /tf \
        /tf_static \
        /estimate/aprilTags/pose \
        /estimate/aprilTags/pose/camera_frame \
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/aprilTags/num_tags_detected \
        /anafi/ned_pose_from_gnss \
        /anafi/gnss_ned_in_body_frame \
        /anafi/gnss_ned_in_body_frame/1hz \
        /estimate/ekf/velocity \
        /anafi/link_goodput \
        /anafi/link_quality \
        /anafi/wifi_rssi"


ANAFI_OUTPUT_TOPICS="/anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/link_goodput \
        /anafi/link_quality \
        /anafi/wifi_rssi \
        /anafi/battery \
        /anafi/state \
        /anafi/pose \
        /anafi/odometry \
        /anafi/rpy \
        /anafi/polled_body_velocities
        /tf"

ANAFI_CMD_TOPICS="/anafi/cmd_takeoff \
        /anafi/cmd_land \
        /anafi/cmd_emergency \
        /anafi/cmd_rpyt \
        /anafi/cmd_moveto \
        /anafi/cmd_moveby \
        /anafi/cmd_camera"

DARKNET_TOPICS="/darknet_ros/bounding_boxes"

ESTIMATE_TOPICS="/estimate/dnn_cv/heading \
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/tcv/pose"

QUAlISYS_TOPICS="/qualisys/Anafi/odom \
        /qualisys/Anafi/pose \
        /qualisys/Anafi/velocity \
        /qualisys/Platform/odom \
        /qualisys/Platform/pose \
        /qualisys/Platform/velocity"

GNC_TOPICS="/guidance/pure_pursuit/velocity_reference \
        /guidance/pid/velocity_reference"

STANDARD_TOPICS="$ANAFI_OUTPUT_TOPICS \
        $ANAFI_CMD_TOPICS \
        $DARKNET_TOPICS \
        $ESTIMATE_TOPICS \
        $GNC_TOPICS \
        /tf" 

if [[ $ENV == "sim" ]]; then
    echo "Rosbagging sim topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $STANDARD_TOPICS 
elif [[ $ENV == "lab" ]]; then
    echo "Rosbagging lab topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $ANAFI_PERCEPTION_RELEVANT_TOPICS \
        $QUAlISYS_TOPICS 
elif [[ $ENV == "eval" ]]; then
    echo "Rosbagging lab topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $EVAL_PIX2GEO
elif [[ $ENV == "real" ]]; then
    echo "Rosbagging real topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $OUTSIDE
fi
