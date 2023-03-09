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

from water_segmentation.srv import sendMask, sendMaskResponse, sendMaskRequest
import sensor_msgs.msg
from geometry_msgs.msg import PointStamped

from map_segmentation_utils import utils



class Segmentation_master:

	def __init__(self, config_file=None):

		rospy.init_node("Segmentation_master", anonymous=False)

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
		self.bridge = CvBridge()
		self._intervals = self.config["settings"]["interval_between_generation"]
		self._use_offline_map_segmentation = self.config["settings"]["enable_offline_map_segmentation"]
		self._use_dl_segmentation = self.config["settings"]["enable_dl_segmentation"]

		self._img_width = rospy.get_param("/drone/camera/img_width")
		self._img_height = rospy.get_param("/drone/camera/img_height")
		self._camera_resolution = (self._img_width, self._img_height)
		self._camera_hfov = rospy.get_param("/drone/camera/camera_hfov")

		self._camera_fov = utils.get_fov_from_hfov(
			self._img_width,
			self._img_height,
			self._camera_hfov
		)

		self._last_gnss_pos = None
		
		rospy.loginfo("[Segmenation master]: All parameters loaded!")


	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["gnss"], 
			sensor_msgs.msg.NavSatFix, 
			self._new_gnss_callback
		)

	def _initalize_services(self):
		self._map_seg_service_name = self.config["services"]["map_seg_name"]
		self._dl_seg_service_name = self.config["services"]["dl_seg_name"]

		if self._use_offline_map_segmentation:
			# wait for the services to become available
			try:
				rospy.wait_for_service(self._map_seg_service_name, timeout=5)
			except:
				rospy.logerr(f"[Segmentation master] Couldt not detect the ROS-service names: {self._map_seg_service_name}. -- > Exiting")
				rospy.signal_shutdown("Service not detected!")

			# create a proxy for the services
			self._map_mask_service_proxy = rospy.ServiceProxy(self._map_seg_service_name, sendMask)

		if self._use_dl_segmentation:
			# wait for the services to become available
			try:
				rospy.wait_for_service(self._dl_seg_service_name, timeout=5)
			except:
				rospy.logerr(f"[Segmentation master] Couldt not detect the ROS-service names: {self._dl_seg_service_name}. -- > Exiting")
				rospy.signal_shutdown("Service not detected!")

			# create a proxy for the services
			self._dl_mask_service_proxy = rospy.ServiceProxy(self._dl_seg_service_name, sendMask)

		self._timer = rospy.Timer(rospy.Duration(self._intervals), self._timer_callback)


	def _setup_publishers(self):

		self.mask_pub = rospy.Publisher(
			self.config["topics"]["output"]["safe_points"], 
			PointStamped, 
			queue_size=10
		)

		self.mask_pub = rospy.Publisher(
			"DEBUG", 
			sensor_msgs.msg.Image, 
			queue_size=10
		)

	def _new_gnss_callback(self, gnss_msg):
		self._last_gnss_pos = [
			gnss_msg.latitude,
			gnss_msg.longitude,
			gnss_msg.altitude
		]
	

	def _timer_callback(self, event):
		# create a request object
		request = sendMaskRequest()
		request.image_request = True

		if self._use_offline_map_segmentation:
			# call the service and get the response
			response = self._map_mask_service_proxy(request)

			# process the response
			map_mask_msg = response.image_data
			map_mask_image = self.bridge.imgmsg_to_cv2(map_mask_msg, "mono8")


		if self._use_dl_segmentation:
			start_time = time.time()
			# call the service and get the response
			response = self._dl_mask_service_proxy(request)
			rospy.loginfo("Time taken: {:.2f} seconds".format(time.time() - start_time))
			# process the response
			dl_mask_msg = response.image_data
			dl_mask_image = self.bridge.imgmsg_to_cv2(dl_mask_msg, "mono8")


		if self._use_dl_segmentation and self._use_offline_map_segmentation:
			# Some fancy logic to determine which one to trust!

			# mask = union_mask
			pass

		elif self._use_offline_map_segmentation:
			mask = map_mask_image

		elif self._use_dl_segmentation:
			mask = dl_mask_image

		
		# Some code down here to determine safe locations for the drone in camera coordinates.
		# Using similar triangles with the focal length probably best shot





	def _shutdown(self):
		rospy.loginfo("[Segmentation master] Shutting down node")

	def start(self):
		rospy.loginfo("[Segmentation master] Starting node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		Segmentation_worker = Segmentation_master()
		Segmentation_worker.start()

if __name__ == "__main__":
		main()
