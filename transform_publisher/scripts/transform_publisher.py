#!/usr/bin/env python3

import logging
import rospy
import tf2_ros as tf2
import tf_conversions
import std_msgs.msg
import numpy as np

from scipy.spatial.transform import Rotation

import tf2_geometry_msgs.tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped

from anafi_uav_msgs.msg import Float32Stamped


class transformPublisher():

	def __init__(self, config_file=None):

		rospy.init_node("transform_publisher", anonymous=False)


		rospy.Subscriber("/anafi/ned_pos_from_gnss", PointStamped,
			self._new_point_callback
		)

		rospy.Subscriber("/anafi/pose", PoseStamped,
			self._new_pose_callback
		)

		rospy.Subscriber("/anafi/height", Float32Stamped,
			self._new_height_cb
		)

		self.gnss_bodyframe_publisher = rospy.Publisher(
			"/anafi/gnss_ned_in_body_frame", PointStamped, queue_size=10
		)

		self.camera_offset_x_mm = rospy.get_param("/drone/camera/offset_x_mm")

		self.tfBuffer = tf2.Buffer()
		self.listener = tf2.TransformListener(self.tfBuffer)

		self.dynamic_broadcaster = tf2.TransformBroadcaster()
		self.static_broadcatser = tf2.StaticTransformBroadcaster()

		self._init_static_broadcaster()

	def _new_height_cb(self, msg):
		# This function as a lookuptable for the drone reported altitude
		t = self.create_transforms(
			from_frame="drone_alt",
			to_frame="ground_alt",
			translation_vec=(0,0,msg.data),
			rotation_quat=(0,0,0,1),
			timestamp=msg.header.stamp
		)

		self.dynamic_broadcaster.sendTransform(t)

	def _new_pose_callback(self, msg):
		# Extract the roll, pitch, and yaw angles from the drone pose
		quaternions=[
			msg.pose.orientation.x,
			msg.pose.orientation.y,
			msg.pose.orientation.z,
			msg.pose.orientation.w
		]
		
		rot = Rotation.from_quat(quaternions)
		(roll, pitch, yaw) = rot.as_euler('xyz', degrees=True)

		# Camera at body origin to body
		camera_at_body_origin_to_body_quat = Rotation.from_euler('xy', [roll, pitch], degrees=True).as_quat()

		t = self.create_transforms(
			from_frame="camera_at_body_origin",
			to_frame="body",
			translation_vec=(0,0,0),
			rotation_quat=camera_at_body_origin_to_body_quat,
			timestamp=msg.header.stamp
		)

		self.dynamic_broadcaster.sendTransform(t)


		# Camera to body origin to world no trans
		camera_at_body_origin_to_world_no_trans_quat = Rotation.from_euler('z', -yaw, degrees=True).as_quat()

		t = self.create_transforms(
			from_frame="camera_at_body_origin",
			to_frame="world_no_trans",
			translation_vec=(0,0,0),
			rotation_quat=camera_at_body_origin_to_world_no_trans_quat,
			timestamp=msg.header.stamp
		)

		self.dynamic_broadcaster.sendTransform(t)

	def _init_static_broadcaster(self):
		camera_to_camera_at_body_origin_quat = Rotation.from_euler('z', -90, degrees=True).as_quat()

		t = self.create_transforms(
			from_frame="camera",
			to_frame="camera_at_body_origin",
			translation_vec=(0, self.camera_offset_x_mm / 1000, 0), # x translation on y axis due to the 90 deg rotation
			rotation_quat=camera_to_camera_at_body_origin_quat,
			timestamp=rospy.Time.now() # Static BR -> time does not matter
		)

		self.static_broadcatser.sendTransform(t)


	def _new_point_callback(self, msg):
		# Construct the translation part of the world -> body transformation
		t = self.create_transforms(
			from_frame="world_no_trans",
			to_frame="world",
			translation_vec= (-msg.point.x, -msg.point.y, -msg.point.z), 
			rotation_quat=(0,0,0,1), # Unit quaternion
			timestamp=msg.header.stamp
		)

		self.dynamic_broadcaster.sendTransform(t)

		# Construct the drone NED position given in the body frame.
		point = PointStamped()
		point.header.frame_id = "world"
		point.header.stamp = msg.header.stamp

		point.point.x = 0
		point.point.y = 0
		point.point.z = 0

		try:
			body_point = self.tfBuffer.transform(point, "body", rospy.Duration(0.1))
		except Exception as e:
			return

		self.gnss_bodyframe_publisher.publish(body_point)

	def create_transforms(self, from_frame, to_frame, translation_vec, rotation_quat, timestamp):
		t = TransformStamped()

		t.header.stamp = timestamp
		t.header.frame_id = from_frame # Transformation from frame id TO child id
		t.child_frame_id = to_frame
		t.transform.translation.x = translation_vec[0]
		t.transform.translation.y = translation_vec[1]
		t.transform.translation.z = translation_vec[2]

		t.transform.rotation.x = rotation_quat[0]
		t.transform.rotation.y = rotation_quat[1]
		t.transform.rotation.z = rotation_quat[2]
		t.transform.rotation.w = rotation_quat[3]

		return t


	def start(self):

		rospy.loginfo("Starting transform publisher")

		while not rospy.is_shutdown():
			rospy.spin()
			



def main():
	estimator = transformPublisher()
	estimator.start()

if __name__ == "__main__":
	main()