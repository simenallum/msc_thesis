#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from geometry_msgs.msg import PointStamped
from perception_master.msg import DetectedPerson
from pix2geo.msg import TrackWorldCoordinate

'''
	ONLY USED FOR DEBUGGING.
	This file is only used to convert all custom messages to std message formats that Rviz can plot. 
'''

class custom2std:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("custom2std_msg", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file or deepsort_params is None:
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

	def _initalize_parameters(self):
		self._confidence_threshold = self.config["settings"]["confidence_threshold"]
		self._floating_object_perception_master_threshold = self.config["settings"]["floating_object_perception_master_threshold"]

		

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["tracks_raw"], 
			TrackWorldCoordinate, 
			self._new_tracks_callback
		)
		
		rospy.Subscriber(
			self.config["topics"]["input"]["perception_detected_humans"], 
			DetectedPerson, 
			self._new_perception_detected_humans_callback
		)

	def _setup_publishers(self):
		self._humans_raw_pub = rospy.Publisher(
			self.config["topics"]["output"]["humans_raw"], 
			PointStamped, 
			queue_size=10
		)

		self._floating_objects_raw_pub = rospy.Publisher(
			self.config["topics"]["output"]["floating_objects_raw"], 
			PointStamped, 
			queue_size=10
		)

		self._perception_floating_objects_pub = rospy.Publisher(
			self.config["topics"]["output"]["perception_floating_objects"], 
			PointStamped, 
			queue_size=10
		)

		self._perception_detected_humans_pub = rospy.Publisher(
			self.config["topics"]["output"]["perception_detected_humans"], 
			PointStamped, 
			queue_size=10
		)

	def _new_tracks_callback(self, msg):
		if msg.class_name == "human":
			self._publish_human_raw([msg.x, msg.y, msg.z], msg.probability)

		if msg.class_name == "floating-object":
			self._publish_floating_object_raw([msg.x, msg.y, msg.z], msg.probability)
		

	def _new_perception_detected_humans_callback(self, point_msg):
		self._publish_perception_humans([point_msg.position.x, point_msg.position.y, point_msg.position.z])
	
	def _prepare_pointstamped_message(self, point):
		msg = PointStamped()
		msg.header.frame_id = "world"
		msg.header.stamp = rospy.Time.now()

		msg.point.x = point[0]
		msg.point.y = point[1]
		msg.point.z = point[2]

		return msg

	def _publish_floating_object_raw(self, world_coordinates, confidence):
		if confidence >= self._floating_object_perception_master_threshold:
			msg = self._prepare_pointstamped_message(world_coordinates)
			self._perception_floating_objects_pub.publish(msg)
		elif confidence >= self._confidence_threshold:
			msg = self._prepare_pointstamped_message(world_coordinates)
			self._floating_objects_raw_pub.publish(msg)

	def _publish_human_raw(self, world_coordinates, confidence):
		if confidence >= self._confidence_threshold:
			msg = self._prepare_pointstamped_message(world_coordinates)
			self._humans_raw_pub.publish(msg)

	def _publish_perception_humans(self, world_coordinates):
		msg = self._prepare_pointstamped_message(world_coordinates)
		self._perception_detected_humans_pub.publish(msg)


	def _shutdown():
		pass

	def start(self):
		rospy.loginfo("Starting custom2std_msg node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		converter = custom2std()
		converter.start()

if __name__ == "__main__":
		main()
