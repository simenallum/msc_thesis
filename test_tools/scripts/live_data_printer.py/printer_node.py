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
		pass

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
			self.config["topics"]["input"]["anafi_pose"], 
			Vector3Stamped, 
			self._new_anafi_rpy_cb
		)

	def _new_gt_pose_cb(self, msg):
		quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

		rot = Rotation.from_quat(quat)
		rot_euler = rot.as_euler('xyz', degrees=True)

		print(f"GT quaternions: {rot_euler[0]}, {rot_euler[1]}, {rot_euler[2]} \n")

	def _new_anafi_pose_cb(self, msg):
		quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

		rot = Rotation.from_quat(quat)
		rot_euler = rot.as_euler('xyz', degrees=True)

		print(f"Anafi quaternions: {rot_euler[0]}, {rot_euler[1]}, {rot_euler[2]} \n")

	def _new_anafi_rpy_cb(self, msg):
		print(f"Anafi rpy: {np.rad2deg(msg.vector.x)}, {np.rad2deg(msg.vector.y)}, {np.rad2deg(msg.vector.z)} \n")


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
