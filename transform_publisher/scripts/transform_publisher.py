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

		self.tfBuffer = tf2.Buffer()
		self.listener = tf2.TransformListener(self.tfBuffer)

		self.dynamic_broadcaster = tf2.TransformBroadcaster()
		self.static_broadcatser = tf2.StaticTransformBroadcaster()

		self._init_static_broadcaster()

	def _new_height_cb(self, msg):
		t = TransformStamped()

		t.header.stamp = msg.header.stamp
		t.header.frame_id = "drone_alt" # Transformation from frame id TO child id
		t.child_frame_id = "ground_alt"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = msg.data

		t.transform.rotation.x = 1
		t.transform.rotation.y = 0
		t.transform.rotation.z = 0
		t.transform.rotation.w = 0

		self.dynamic_broadcaster.sendTransform(t)

	def _new_pose_callback(self, msg):
		quaternions=[
			msg.pose.orientation.x,
			msg.pose.orientation.y,
			msg.pose.orientation.z,
			msg.pose.orientation.w
		]
		
		rot = Rotation.from_quat(quaternions)
		(roll, pitch, yaw) = rot.as_euler('xyz', degrees=True)

		world_to_drone = Rotation.from_euler('xy', [roll, pitch], degrees=True) # Dont mind the yaw angle
		camera_to_drone = world_to_drone

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

		world_to_drone = Rotation.from_euler('z', -yaw, degrees=True)
		quat = world_to_drone.as_quat()


		t = TransformStamped()

		t.header.stamp = msg.header.stamp
		t.header.frame_id = "camera_at_body_origin" # Transformation from frame id TO child id
		t.child_frame_id = "world_no_trans"
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

		t = TransformStamped()

		t.header.stamp = msg.header.stamp
		t.header.frame_id = "world_no_trans" # Transformation from frame id TO child id
		t.child_frame_id = "world"
		t.transform.translation.x = -msg.point.x # Minus her??? 
		t.transform.translation.y = -msg.point.y
		t.transform.translation.z = -msg.point.z

		t.transform.rotation.x = 0
		t.transform.rotation.y = 0
		t.transform.rotation.z = 0
		t.transform.rotation.w = 1

		self.dynamic_broadcaster.sendTransform(t)

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


	def start(self):

		rospy.loginfo("Starting transform publisher")
		rate = rospy.Rate(1) # 0.2 Hz, equivalent to 5 seconds
		rate.sleep()

		while not rospy.is_shutdown():
			point = PointStamped()
			point.header.frame_id = "camera"
			point.header.stamp = rospy.Time.now() - rospy.Duration(0.1)

			point.point.x = 0
			point.point.y = 0
			point.point.z = 1


			body_point = self.tfBuffer.transform(point, "world", rospy.Duration(1))


			print(body_point)


			rate.sleep()
			



def main():
	estimator = transformPublisher()
	estimator.start()

if __name__ == "__main__":
	main()