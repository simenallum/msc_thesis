#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PointStamped
from pix2geo.msg import TrackWorldCoordinate
from anafi_uav_msgs.msg import PointWithCovarianceStamped
from perception_master.msg import DetectedPerson


class Perception_master:

	def __init__(self, config_file=None):

		rospy.init_node("Perception_master", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file is None:
			config_file = rospy.get_param("~config_file")

		try:
			with open(f"{script_dir}/../config/{config_file}") as f:
				self.config = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		self._initalize_parameters()
		self._setup_publishers()
		self._setup_subscribers()
		self._initalize_services()

	def _initalize_parameters(self):
		
		
		rospy.loginfo("[Segmenation master]: All parameters initalized!")


	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["safe_points_world_coord"], 
			PointStamped, 
			self._new_safe_point_callback
		)
		
		rospy.Subscriber(
			self.config["topics"]["input"]["tracks_world_coord"], 
			TrackWorldCoordinate, 
			self._new_track_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["platform_EKF"], 
			PointWithCovarianceStamped, 
			self._new_platform_EKF_callback
		)

	def _initalize_services(self):
		pass


	def _setup_publishers(self):
		self._detected_person_pub = rospy.Publisher(
			self.config["topics"]["output"]["detected_person"], 
			DetectedPerson, 
			queue_size=10
		)

		self._safe_point_pub = rospy.Publisher(
			self.config["topics"]["output"]["safe_point"], 
			PointStamped, 
			queue_size=10
		)

	def _new_safe_point_callback(self, safe_point_msg):
		pass

	def _new_track_callback(self, track_msg):
		pass

	def _new_platform_EKF_callback(self, ekf_msg):
		pass


	def _shutdown(self):
		rospy.loginfo("[Perception master] Shutting down node")

	def start(self):
		rospy.loginfo("[Perception master]: Starting node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		master = Perception_master()
		master.start()

if __name__ == "__main__":
		main()
