#!/usr/bin/env python3

import logging
import rospy
import tf2_ros as tf2
import tf_conversions
import std_msgs.msg
import numpy as np

from scipy.spatial.transform import Rotation

import tf2_geometry_msgs.tf2_geometry_msgs

from geometry_msgs.msg import QuaternionStamped, TransformStamped, PointStamped


class transformPublisher():

	def __init__(self, config_file=None):

		rospy.init_node("transform_publisher", anonymous=False)


		rospy.Subscriber("/anafi/ned_pos_from_gnss", PointStamped,
			self._new_point_callback
		)

		rospy.Subscriber("/anafi/attitude", QuaternionStamped,
			self._new_pose_callback
		)

		self.gnss_bodyframe_publisher = rospy.Publisher(
			"/anafi/gnss_ned_in_body_frame", PointStamped, queue_size=10
		)

		self.tfBuffer = tf2.Buffer()
		self.listener = tf2.TransformListener(self.tfBuffer)

		self.dynamic_broadcaster = tf2.TransformBroadcaster()
		self.static_broadcatser = tf2.StaticTransformBroadcaster()

		self._init_static_broadcaster()

	def _new_pose_callback(self, msg):
		quaternions=[
			msg.quaternion.x,
			msg.quaternion.y,
			msg.quaternion.z,
			msg.quaternion.w
		]
		
		rot = Rotation.from_quat(quaternions)
		(roll, pitch, yaw) = rot.as_euler('xyz', degrees=True)

		world_to_drone = Rotation.from_euler('xy', [-roll, -pitch], degrees=True) # Dont mind the yaw angle
		camera_to_drone = world_to_drone.inv()

		quaternions = camera_to_drone.as_quat()

		t = TransformStamped()

		t.header.stamp = msg.header.stamp
		t.header.frame_id = "camera_at_body_origin" # Transformation from frame id TO child id
		t.child_frame_id = "body"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.0

		t.transform.rotation.x = quaternions[0]
		t.transform.rotation.y = quaternions[1]
		t.transform.rotation.z = quaternions[2]
		t.transform.rotation.w = quaternions[3]

		self.dynamic_broadcaster.sendTransform(t)

		world_to_drone = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True) # Down mind the yaw angle
		quat = world_to_drone.inv().as_quat()

		t = TransformStamped()

		t.header.stamp = msg.header.stamp
		t.header.frame_id = "body" # Transformation from frame id TO child id
		t.child_frame_id = "world"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.0

		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]

		self.dynamic_broadcaster.sendTransform(t)

	def _init_static_broadcaster(self):
		static_tr_stamped = TransformStamped()
		static_tr_stamped.header.stamp = rospy.Time.now()

		static_tr_stamped.header.frame_id = "camera" # Transformation from frame id TO child id
		static_tr_stamped.child_frame_id = "camera_at_body_origin"

		static_tr_stamped.transform.translation.x = 0.0
		static_tr_stamped.transform.translation.y = 0.07
		static_tr_stamped.transform.translation.z = 0.0

		camera_to_body_rotation = Rotation.from_euler('z', -90, degrees=True)
		camera_to_body_quat = camera_to_body_rotation.as_quat()
		static_tr_stamped.transform.rotation.x = camera_to_body_quat[0]
		static_tr_stamped.transform.rotation.y = camera_to_body_quat[1]
		static_tr_stamped.transform.rotation.z = camera_to_body_quat[2]
		static_tr_stamped.transform.rotation.w = camera_to_body_quat[3]

		self.static_broadcatser.sendTransform(static_tr_stamped)


	def _new_point_callback(self, msg):
		
		point = PointStamped()
		point.header.frame_id = msg.header.frame_id
		point.header.stamp = msg.header.stamp

		point.point.x = -msg.point.x
		point.point.y = -msg.point.y
		point.point.z = -msg.point.z

		try:
			body_point = self.tfBuffer.transform(point, "body", rospy.Duration(0.1))
		except Exception as e:
			return

		self.gnss_bodyframe_publisher.publish(body_point)


	def start(self):

		rospy.loginfo("Starting transform publisher")

		while not rospy.is_shutdown():
			rospy.spin()
			



def main():
	estimator = transformPublisher()
	estimator.start()

if __name__ == "__main__":
	main()