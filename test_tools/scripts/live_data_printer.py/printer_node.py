#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from scipy.spatial.transform import Rotation

import pymap3d


class DataPrinter:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("DataPrinter", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file or deepsort_params is None:
			config_file = rospy.get_param("~config_file")

		try:
			with open(f"{script_dir}/../../config/{config_file}") as f:
				self.config = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		self._initalize_parameters()
		self._setup_subscribers()

	def _initalize_parameters(self):
		self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

		self.last_gt_quat = [0, 0, 0]
		self.last_anafi_quat = [0, 0, 0]
		self.last_anafi_rpy = [0, 0, 0]


	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["GT_pose"], 
			PoseStamped, 
			self._new_gt_pose_cb
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["anafi_pose"], 
			PoseStamped, 
			self._new_anafi_pose_cb
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["anafi_rpy"], 
			Vector3Stamped, 
			self._new_anafi_rpy_cb
		)

	def timer_callback(self, timer):
		print("-------------------------")
		print(f"GT quaternions: {self.last_gt_quat[0]}, {self.last_gt_quat[1]}, {self.last_gt_quat[2]}")
		print(f"Anafi quaternions: {self.last_anafi_quat[0]}, {self.last_anafi_quat[1]}, {self.last_anafi_quat[2]}")
		print(f"Anafi rpy: {self.last_anafi_rpy[0]}, {self.last_anafi_rpy[1]}, {self.last_anafi_rpy[2]}")
		print("-------------------------")


	def _new_gt_pose_cb(self, msg):
		quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

		rot = Rotation.from_quat(quat)
		rot_euler = rot.as_euler('xyz', degrees=True)

		self.last_gt_quat = [rot_euler[0], rot_euler[1] ,rot_euler[2]]

	def _new_anafi_pose_cb(self, msg):
		quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

		rot = Rotation.from_quat(quat)
		rot_euler = rot.as_euler('xyz', degrees=True)

		self.last_anafi_quat = [rot_euler[0], rot_euler[1] ,rot_euler[2]]

	def _new_anafi_rpy_cb(self, msg):
		self.last_anafi_rpy = [np.rad2deg(msg.vector.x), np.rad2deg(msg.vector.y), np.rad2deg(msg.vector.z)]


	def _shutdown():
		rospy.loginfo("Shutting down DataPrinter node")

	def start(self):
		rospy.loginfo("Starting DataPrinter node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		printer = DataPrinter()
		printer.start()

if __name__ == "__main__":
		main()
