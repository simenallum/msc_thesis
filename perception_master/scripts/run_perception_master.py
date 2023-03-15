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
from std_srvs import SetBool, SetBoolRequest, SetBoolResponse

from perception_master_utils import utils

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
		# Platform detection utils
		self._min_dist_to_initiate_platform_tracking = self.config["settings"]["min_dist_to_initiate_platform_tracking"]

		# Human and FO detection
		self._radius_of_acceptance_new_human_detections = self.config["settings"]["radius_of_acceptance_new_human_detections"]
		self._radius_of_acceptance_new_FO_detections = self.config["settings"]["radius_of_acceptance_new_FO_detections"]
		self._critical_level_distances = self.config["settings"]["critical_level_distances"]
		self._track_confidence_threshold = self.config["settings"]["track_confidence_threshold"]
		self._human_table = {}
		self._FO_table = {}

		# Safe point generation
		self._radius_of_acceptance_new_safe_points = self.config["settings"]["radius_of_acceptance_new_safe_points"]
		self._safe_points_table = []

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
		self._AT_node_activation_name = self.config["services"]["AT_node_activation_name"]
		self._DNN_node_activation_name = self.config["services"]["DNN_node_activation_name"]
		self._GNSS_node_activation_name = self.config["services"]["GNSS_node_activation_name"]

		# wait for the services to become available
		try:
			rospy.wait_for_service(self._AT_node_activation_name, timeout=5)
			rospy.wait_for_service(self._DNN_node_activation_name, timeout=5)
			rospy.wait_for_service(self._GNSS_node_activation_name, timeout=5)
		except:
			rospy.logerr(f"[Perception master] Couldt not detect one of the activation nodes ROS services -- > Exiting")
			rospy.signal_shutdown("Service not detected!")

		self._activate_AT_node_proxy = rospy.ServiceProxy(self._AT_node_activation_name, SetBool)
		self._activate_DNN_node_proxy = rospy.ServiceProxy(self._DNN_node_activation_name, SetBool)
		self._activate_GNSS_node_proxy = rospy.ServiceProxy(self._GNSS_node_activation_name, SetBool)


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

		new_safe_point = self._extract_point_msg(safe_point_msg)
		point_in_table = utils.is_point_within_threshold_list(
								new_safe_point, 
							   	self._radius_of_acceptance_new_safe_points,
								self._safe_points_table)
		
		if not point_in_table:
			self._safe_points_table.append(new_safe_point)

			self._publish_safe_point(new_safe_point, safe_point_msg.header)

	def _new_track_callback(self, track_msg):
		track = self._extract_track_msg(track_msg)

		# Only handle track if confidence is high enough
		if track["confidence"] <= self._track_confidence_threshold:
			return

		# Handle human tracks
		if track["class_name"] == "human":

			# Track_id already exists
			if utils.key_exists(track["track_id"], self._human_table):

				distance_between_measurements = utils.calculate_euclidian_distance(track["point"], self._human_table["track_id"])
				if distance_between_measurements >= self._radius_of_acceptance_new_human_detections:
					# Update position for track
					self._human_table["track_id"] = track["point"]

			# Track ID does not exist
			else:
				point_in_table = utils.is_point_within_threshold_dict(track["point"], self._radius_of_acceptance_new_human_detections, self._human_table)
				
				if not point_in_table:
					# Add new track and point
					self._human_table[track["track_id"]] = track["point"]

			
			distance_to_FO = utils.get_closest_distance(track["point"], self._FO_table)
			critical_level = utils.calculate_critical_level(distance_to_FO, self._critical_level_distances)
			
			self._publish_humans_detection(track["point"], critical_level, track_msg.header)
					

		# Handle FO tracks
		elif track["class_name"] == "floating-objects":
			# Track_id already exists
			if utils.key_exists(track["track_id"], self._FO_table):
				distance_between_measurements = utils.calculate_euclidian_distance(track["point"], self._FO_table["track_id"])
				if distance_between_measurements >= self._radius_of_acceptance_new_FO_detections:
					self._FO_table["track_id"] = track["point"]

			else:
				point_in_table = utils.is_point_within_threshold_dict(track["point"], self._radius_of_acceptance_new_FO_detections, self._FO_table)
				if not point_in_table:
					self._FO_table[track["track_id"]] = track["point"]


	def _new_platform_EKF_callback(self, ekf_msg):
		vector = self._extract_ekf_msg(ekf_msg)

		length = utils.calculate_euclidian_distance_of_vector(vector)

		if length > self._min_dist_to_initiate_platform_tracking:
			# create a request object
			request = SetBoolRequest()
			request.data = True

			response = self._activate_GNSS_node_proxy(request)

			request = SetBoolRequest()
			request.data = False

			response = self._activate_AT_node_proxy(request)
			response = self._activate_DNN_node_proxy(request)

		else:
			# create a request object
			request = SetBoolRequest()
			request.data = True

			response = self._activate_AT_node_proxy(request)
			response = self._activate_DNN_node_proxy(request)

			request = SetBoolRequest()
			request.data = False

			response = self._activate_GNSS_node_proxy(request)

			


	def _extract_point_msg(self, point_msg):
		return np.array([point_msg.point.x, point_msg.point.y, point_msg.point.z])
	
	def _extract_track_msg(self, track_msg):
		content = {
			"point": np.array([track_msg.x, track_msg.y, track_msg.z]),
			"track_id": track_msg.track_id,
			"class_name": track_msg.class_name,
			"confidence": track_msg.probabilty
		}

		return content
	
	def _extract_ekf_msg(self, ekf_msg):
		return np.array([ekf_msg.position.x, ekf_msg.position.y, ekf_msg.position.z])
	
	def _publish_safe_point(self, safe_point, header):
		safe_point_message = PointStamped()
		safe_point_message.header = header
		safe_point_message.point.x = safe_point[0]
		safe_point_message.point.y = safe_point[1]
		safe_point_message.point.z = safe_point[2]

		self._safe_point_pub.publish(safe_point_message)

	def _publish_humans_detection(self, human_point, critical_level, header):
		human_detection_msg = DetectedPerson()
		human_detection_msg.header = header
		human_detection_msg.position.x = human_point[0]
		human_detection_msg.position.y = human_point[1]
		human_detection_msg.position.z = human_point[2]
		human_detection_msg.severity = critical_level

		self._detected_person_pub.publish(human_detection_msg)

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
