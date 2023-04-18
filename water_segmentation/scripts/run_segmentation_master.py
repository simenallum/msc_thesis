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
from std_msgs.msg import Float64
from anafi_uav_msgs.msg import Float32Stamped

from segmentation_master_utils import utils
import matplotlib.pyplot as plt



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
		self._stride = self.config["settings"]["stride_in_search"]
		self._use_offline_map_segmentation = self.config["settings"]["enable_offline_map_segmentation"]
		self._use_dl_segmentation = self.config["settings"]["enable_dl_segmentation"]
		self._min_altitude_to_generate_safe_points = self.config["settings"]["min_altitude_to_generate_safe_points"]
		self._min_altitude_to_use_dl_segmentation = self.config["settings"]["min_altitude_to_use_dl_segmentation"]

		self._dice_threshold = self.config["settings"]["dice_threshold"]
		self._safe_metric_dist = self.config["settings"]["min_safe_metric_dist"]

		self._debug = self.config["debug"]
		if self._debug:
			self._DL_mask_usage_counter = []

		self._img_width = rospy.get_param("/drone/camera/img_width")
		self._img_height = rospy.get_param("/drone/camera/img_height")
		self._camera_resolution = (self._img_width, self._img_height)
		self._camera_hfov = rospy.get_param("/drone/camera/camera_hfov")
		self._K = np.array(rospy.get_param("/drone/camera/camera_matrix")).reshape(3,3)
		self._focal_length = (self._K[0,0] + self._K[1,1])/2

		self._camera_fov = utils.get_fov_from_hfov(
			self._img_width,
			self._img_height,
			self._camera_hfov
		)

		self._last_height_meas = [None]
		
		rospy.loginfo("[Segmenation master]: All parameters loaded!")


	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["height"], 
			Float32Stamped, 
			self._new_height_CB
		)

	def _initalize_services(self):
		self._map_seg_service_name = self.config["services"]["map_seg_name"]
		self._dl_seg_service_name = self.config["services"]["dl_seg_name"]

		if self._use_offline_map_segmentation:
			# wait for the services to become available
			try:
				rospy.wait_for_service(self._map_seg_service_name, timeout=10)
			except:
				rospy.logerr(f"[Segmentation master] Couldt not detect the ROS-service names: {self._map_seg_service_name}. -- > Exiting")
				rospy.signal_shutdown("Service not detected!")

			# create a proxy for the services
			self._map_mask_service_proxy = rospy.ServiceProxy(self._map_seg_service_name, sendMask)

		if self._use_dl_segmentation:
			# wait for the services to become available
			try:
				rospy.wait_for_service(self._dl_seg_service_name, timeout=10)
			except:
				rospy.logerr(f"[Segmentation master] Couldt not detect the ROS-service names: {self._dl_seg_service_name}. -- > Exiting")
				rospy.signal_shutdown("Service not detected!")

			# create a proxy for the services
			self._dl_mask_service_proxy = rospy.ServiceProxy(self._dl_seg_service_name, sendMask)

		self._timer = rospy.Timer(rospy.Duration(self._intervals), self._timer_callback)


	def _setup_publishers(self):

		self.safe_point_pub = rospy.Publisher(
			self.config["topics"]["output"]["safe_points"], 
			PointStamped, 
			queue_size=10
		)

		if self._debug:
			self.mask_pub = rospy.Publisher(
				"SEGMASK_with_SP", 
				sensor_msgs.msg.Image, 
				queue_size=10
			)

			self.dice_score_pub = rospy.Publisher(
				"SEGMASK_dice_score",
				Float64,
				queue_size=10
			)

			self.percentage_DL_mask_used_pub = rospy.Publisher(
				"SEGMASK_percentage_DL_mask_used",
				Float64,
				queue_size=10
			)
	
	def _new_height_CB(self, meas):
		self._last_height_meas = [meas.data]

	
	def _timer_callback(self, event):
		if None in self._last_height_meas:
			rospy.logwarn(f"Can not create segmentation mask. Missing height-measurements")
			return

		if self._last_height_meas[0] < self._min_altitude_to_generate_safe_points:
			return
		
		# create a request object
		request = sendMaskRequest()
		request.image_request = True

		if self._use_dl_segmentation:
			start_time = time.time()
			# call the service and get the response
			response = self._dl_mask_service_proxy(request)
			rospy.logdebug("Time taken: {:.2f} seconds".format(time.time() - start_time))
			# process the response
			dl_mask_msg = response.image_data
			dl_mask_image = self.bridge.imgmsg_to_cv2(dl_mask_msg, "mono8")

		if self._use_offline_map_segmentation:
			# call the service and get the response
			response = self._map_mask_service_proxy(request)

			# process the response
			map_mask_msg = response.image_data
			map_mask_image = self.bridge.imgmsg_to_cv2(map_mask_msg, "mono8")

		if self._last_height_meas[0] < self._min_altitude_to_use_dl_segmentation:
			mask = map_mask_image

		elif self._use_dl_segmentation and self._use_offline_map_segmentation:
			# Check if the two masks overlap roughly -> indicates DL seg mask is usable
			pecentage_overlap = utils.compare_image_masks(dl_mask_image, map_mask_image)
			if (pecentage_overlap > self._dice_threshold):
				mask = dl_mask_image

				if self._debug:
					self._DL_mask_usage_counter.append(1)

			# DL map is too risky to use -> use the more safe map seg mask
			else:
				mask = map_mask_image

				if self._debug:
					self._DL_mask_usage_counter.append(0)

			if self._debug:
				msg = Float64()
				msg.data = pecentage_overlap
				self.dice_score_pub.publish(msg)

				count_ones = self._DL_mask_usage_counter.count(1)
				percentage_ones = (count_ones / len(self._DL_mask_usage_counter)) * 100
				msg = Float64()
				msg.data = percentage_ones
				self.percentage_DL_mask_used_pub.publish(msg)

		elif self._use_offline_map_segmentation:
			mask = map_mask_image

		elif self._use_dl_segmentation:
			mask = dl_mask_image

		safe_dist_px = utils.convert_save_dist_to_px(self._focal_length, self._last_height_meas, self._safe_metric_dist)

		if safe_dist_px > min(self._camera_resolution):
			return
			
		start_time = time.time()
		safe_points = utils.find_safe_areas(mask, safe_dist_px, stride=self._stride)
		rospy.loginfo("Time taken to find SP: {:.2f} seconds".format(time.time() - start_time))

		if not (np.any(safe_points) == None):
			self._publish_safe_point(safe_points)

		if self._debug:
			# Convert the mask to an RGB color image
			mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

			if not (np.any(safe_points) == None):
				# Define the center of the circle
				center = (int(safe_points[1]), int(safe_points[0]))

				# Draw a red circle at the center
				cv2.circle(mask_rgb, center, 10, (255, 0, 0), -1)

			# Publish the image as a uint8
			self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_rgb, "rgb8"))


	def _prepare_out_message(self, point):
		msg = PointStamped()
		msg.header.stamp = rospy.Time.now()

		msg.point.x = point[0]
		msg.point.y = point[1]

		return msg

	def _publish_safe_point(self, point):
		msg = self._prepare_out_message(point)
		self.safe_point_pub.publish(msg)


	def _shutdown(self):
		rospy.loginfo("[Segmentation master]: Shutting down node")


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