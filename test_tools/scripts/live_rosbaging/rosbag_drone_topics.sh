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
              /anafi/gnss_location \
              /anafi/height \
              /anafi/optical_flow_velocities \
              /anafi/polled_body_velocities \
              /anafi/pose \
              /anafi/rpy \
              /tf \
              /tf_static \
              /anafi/ned_pos_from_gnss \
              /qualisys/Anafi/pose \
              /qualisys/AT0/pose \
              /search/tracks \
              /search/tracks/camera_coordinates_tria \
              /search/tracks/camera_coordinates_fov \
              /search/tracks/world_coordinates \
              /search/tracks/world_coordinates/fov"

ANAFI_PT_TOPICS="/anafi/image \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/pose \
        /anafi/polled_body_velocities \
        /anafi/optical_flow_velocities \
        /estimate/aprilTags/pose \
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/aprilTags/num_tags_detected \
        /anafi/ned_pos_from_gnss \
        /anafi/gnss_ned_in_body_frame/downsampled"

OUTSIDE="/anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /darknet_ros/bounding_boxes \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/pose \
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
        /anafi/gnss_ned_in_body_frame/downsampled \
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

BAREBONE_TOPICS="/anafi/image \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/pose \
        /anafi/rpy \
        /anafi/ned_pos_from_gnss \
        /anafi/ned_frame_gnss_origin \
        /anafi/optical_flow_velocities \
        /anafi/polled_body_velocities"

TRANSFORM_TOPICS="/tf \
        /tf_static"

SEGMENTATION_TOPICS="/segmentation/safe_points/image_coordinates"

DEBUG_TOPICS="/debug/segmentation_mask_with_sp \
        /debug/segmentation_mask_overlap_percentage \
        /debug/segmentation_dl_mask_used_percentage \
        /debug/segmentation_offline_mask \
        /debug/segmentation_dl_mask \
        /debug/tracks/humans_raw \
        /debug/floating_objects_raw \
        /debug/perception_detected_humans \
        /debug/perception_floating_objects"

PLATFORM_TRACKER_TOPICS="/anafi/gnss_ned_in_body_frame/downsampled \
        /estimate/aprilTags/pose \
        /estimate/dnn_cv/position \
        /estimate/aprilTags/num_tags_detected \
        /estimate/ekf \
        /anafi/height"

STANDARD_TOPICS="$ANAFI_OUTPUT_TOPICS \
        $ANAFI_CMD_TOPICS \
        $ESTIMATE_TOPICS \
        $GNC_TOPICS \
        /tf" 

if [[ $ENV == "sim" ]]; then
    echo "Rosbagging sim topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $STANDARD_TOPICS 
elif [[ $ENV == "lab_eval_pt" ]]; then
    echo "Rosbagging lab topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $ANAFI_PT_TOPICS \
        $QUAlISYS_TOPICS 
elif [[ $ENV == "eval_pix2geo" ]]; then
    echo "Rosbagging lab topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $EVAL_PIX2GEO
elif [[ $ENV == "real" ]]; then
    echo "Rosbagging real topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $OUTSIDE
elif [[ $ENV == "all" ]]; then
    echo "Rosbagging all topics"
    rosbag record -a -O $OUTPUT_DIR/$TIME
elif [[ $ENV == "all_offline" ]]; then
    echo "Rosbagging all topics"
    rosbag record -a -O $OUTPUT_DIR/$TIME
elif [[ $ENV == "eval_pt" ]]; then
    echo "Rosbagging all topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $PLATFORM_TRACKER_TOPICS
elif [[ $ENV == "seg_eval" ]]; then
    echo "Rosbagging segmenation eval topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $EVAL_SEG_TOPICS
elif [[ $ENV == "raw_data" ]]; then
    echo "Rosbagging ONLY RAW TOPICS"
    rosbag record -O $OUTPUT_DIR/$TIME \
        $BAREBONE_TOPICS
fi
