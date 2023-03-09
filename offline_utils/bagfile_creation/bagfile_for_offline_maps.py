#!/usr/bin/env python3

import rospy
import rosbag
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

# Set up ROS node and publisher
rospy.init_node('drone_data_simulator', anonymous=True)

# Set up bagfile
bag = rosbag.Bag('drone_data.bag', 'w')

# Set up simulation parameters
duration = 60.0  # seconds
rate = 30  # Hz
radius = 100.0  # meters
min_altitude = 10.0  # meters
max_altitude = 50.0  # meters
start_time = rospy.Time.now()
init_pos = (63.442088, 10.415013)

# Simulate drone data
for i in range(int(duration * rate)):
    # Calculate time and position
    t = rospy.Duration.from_sec(float(i) / float(rate))
    pos_x = radius * math.cos(2 * math.pi * float(i) / float(duration * rate))
    pos_y = radius * math.sin(2 * math.pi * float(i) / float(duration * rate))
    pos_z = min_altitude + (max_altitude - min_altitude) * float(i) / float(duration * rate)
    yaw_angle = math.atan2(pos_y, pos_x)

    # Generate GNSS position data
    gnss_msg = NavSatFix()
    gnss_msg.header.stamp = start_time + t
    gnss_msg.latitude = init_pos[0] + pos_y / 111319.9
    gnss_msg.longitude = init_pos[1] + pos_x / (111319.9 * math.cos(math.radians(gnss_msg.latitude)))
    gnss_msg.altitude = pos_z
    gnss_msg.position_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
    
    # Generate pose data
    pose_msg = PoseStamped()
    pose_msg.header.stamp = start_time + t
    pose_msg.pose.position.x = pos_x
    pose_msg.pose.position.y = pos_y
    pose_msg.pose.position.z = pos_z
    pose_msg.pose.orientation.w = math.cos(yaw_angle / 2.0)
    pose_msg.pose.orientation.z = math.sin(yaw_angle / 2.0)

    # Record to bagfile
    bag.write('/anafi/gnss_location', gnss_msg, gnss_msg.header.stamp)
    bag.write('/anafi/pose', pose_msg, pose_msg.header.stamp)

bag.close()
