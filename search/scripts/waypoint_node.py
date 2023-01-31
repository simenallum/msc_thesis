#!/usr/bin/env python3

import rospy
import os
import numpy as np
import yaml
import sys
from cv_bridge import CvBridge

from yolov8_ros.msg import BoundingBox, BoundingBoxes
from search_node_utils.search_pattern import get_fov_from_hfov, get_traversal_grid, save_lla_to_file, delete_first_and_return_second, return_first_coordinate
import sensor_msgs.msg
from std_srvs.srv import SetBool, SetBoolResponse


'''
At startup: 

	initialize all parameters:
	- Area size
	- Camera fov
	- Overlap
	- Search altitude

	Initialize in/out:
	- Service: Distressed coordinates
	- Publisher: Next waypoint in GNSS format
	- Subsriber: GNSS position


When service - distressed coordinate - received:
	- Generate waypoint list and save to file.
'''

class WaypointMaster:
	
	def __init__(self, config_file=None):
		
		rospy.init_node("WaypointMaster", anonymous=False)

		self.script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file is None:
			config_file = rospy.get_param("~config_file")

		try:
			with open(f"{self.script_dir}/../config/{config_file}") as f:
				self.config = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		self._initalize_parameters()
		self._setup_publishers()
		self._setup_subscribers()
		self._initalize_services()
		self._generate_search_grid()

	def _initalize_parameters(self):
		self.distressed_coordinates = (self.config["search"]["distressed_coord"]["lat"],
						  			   self.config["search"]["distressed_coord"]["long"])
		self.area_size = (self.config["search"]["areasize"]["north"],
						  self.config["search"]["areasize"]["north"])
		self.search_altitude = self.config["search"]["altitude"]
		self.subgrids_overlap = self.config["search"]["overlap"]
		self.camera_hfov = self.config["camera"]["hfov"]
		self.image_width = self.config["camera"]["image_width"]
		self.image_height = self.config["camera"]["image_height"]

		self.save_grid_subfolder_name = self.config["utilities"]["save_grid"]["subfolder_name"]
		self.save_grid_file_path = f"processing{self.script_dir}/../{self.save_grid_subfolder_name}"

		self.last_gnss = sensor_msgs.msg.NavSatFix()

		self.node_active = False

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["GNSS"], 
			sensor_msgs.msg.NavSatFix, 
			self._new_gnss_callback
		)

	def _initalize_services(self):
		self.srv_node_activation = rospy.Service(
			self.config["service"]["node_activation"],
			SetBool, 
			self._handle_node_activation
		)

	def _setup_publishers(self):
		self._waypoint_pub = rospy.Publisher(
			self.config["topics"]["output"]["waypoint"], 
			sensor_msgs.msg.NavSatFix, 
			queue_size=10
		)

	def _handle_node_activation(self, req):
		if req.data:
			self.node_active = True
			res = SetBoolResponse()
			res.success = True
			res.message = "Started processing data"

			# Publish first node in the waypoint list
			coordinate = return_first_coordinate(self.save_grid_file_path, "backup.txt")
			msg = self._prepare_navsat_message(coordinate)
			self._publish_waypoint(msg)

			return res 
		else:
			self.node_active = False
			res = SetBoolResponse()
			res.success = True
			res.message = "Stopped processing data"
			return res

	def _new_gnss_callback(self, msg):
		self.last_gnss = msg

	def _generate_search_grid(self):
		camera_fov = get_fov_from_hfov(self.image_width, self.image_height, self.camera_hfov)
		waypoints = get_traversal_grid(self.search_altitude, 
			camera_fov, 
			self.subgrids_overlap,
			self.area_size,
			self.distressed_coordinates)

		save_lla_to_file(waypoints, self.save_grid_file_path, "backup.txt")

	def _prepare_navsat_message(self, coordinate):
		latitude = coordinate[0]
		longitude = coordinate[1]
		altitude = coordinate[2]

		navsatfix = sensor_msgs.msg.NavSatFix()
		navsatfix.header.stamp = rospy.Time.now()
		navsatfix.header.frame_id = "body"
		navsatfix.latitude = latitude
		navsatfix.longitude = longitude
		navsatfix.altitude = altitude
		navsatfix.status.status = sensor_msgs.msg.NavSatStatus.STATUS_FIX
		navsatfix.status.service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS
		return navsatfix

	def _publish_waypoint(self, msg):
		self._waypoint_pub.publish(msg)

	def _shutdown():
		rospy.loginfo("Shutting down WaypointMaster")

	def start(self):
		rospy.loginfo("Starting WaypointMaster")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():
			if self.node_active:
				pass


def main():
		SM = WaypointMaster()
		SM.start()

if __name__ == "__main__":
		main()