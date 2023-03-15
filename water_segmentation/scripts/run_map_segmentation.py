#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation

from water_segmentation.srv import sendMask, sendMaskResponse
import sensor_msgs.msg
import geometry_msgs.msg

from map_segmentation_utils import utils



class Map_segmentation:

	def __init__(self, config_file=None):

		rospy.init_node("Map_segmentation", anonymous=False)

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

		self._img_width = rospy.get_param("/drone/camera/img_width")
		self._img_height = rospy.get_param("/drone/camera/img_height")
		self._camera_resolution = (self._img_width, self._img_height)
		self._camera_hfov = rospy.get_param("/drone/camera/camera_hfov")

		self._camera_fov = utils.get_fov_from_hfov(
			self._img_width,
			self._img_height,
			self._camera_hfov
		)

		self._map_resolution = (self.config['offline_map']['map_resolution_px']['width'],
			  					self.config['offline_map']['map_resolution_px']['height'])
		self._large_map_radius = self.config['offline_map']['map_radius']
		self._shapefile_path = self.config['offline_map']['shapefile_path']

		self._shapefile_gpd = utils.read_gdp_from_file(self._shapefile_path)
		
		rospy.loginfo("[Map segmentation]: All parameters loaded!")

		self._last_yaw_angle = None
		self._last_gnss_pos = None


	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["gnss"], 
			sensor_msgs.msg.NavSatFix, 
			self._new_gnss_callback
		)
		
		rospy.Subscriber(
			self.config["topics"]["input"]["drone_pose"], 
			geometry_msgs.msg.PoseStamped, 
			self._new_drone_pose_callback
		)

	def _initalize_services(self):
		self._srv_make_mask = rospy.Service(
			self.config["services"]["make_mask"],
			sendMask, 
			self._handle_create_mask
		)
		

	def _setup_publishers(self):

		self.mask_pub = rospy.Publisher(
			self.config["topics"]["output"]["water_mask"], 
			sensor_msgs.msg.Image, 
			queue_size=10
		)

	def _new_gnss_callback(self, gnss_msg):
		self._last_gnss_pos = [
			gnss_msg.latitude,
			gnss_msg.longitude,
			gnss_msg.altitude
		]
	
	def _new_drone_pose_callback(self, pose_msg):
		quaternions=[
			pose_msg.pose.orientation.x,
			pose_msg.pose.orientation.y,
			pose_msg.pose.orientation.z,
			pose_msg.pose.orientation.w
		]
		
		rot = Rotation.from_quat(quaternions)
		(_, _, yaw) = rot.as_euler('xyz', degrees=True)

		self._last_yaw_angle = yaw

	def _handle_create_mask(self, req):
		# Calculate the ground coverage based on camera field of view and drone altitude
		ground_coverage = utils.calculate_ground_coverage(camera_fov=self._camera_fov, altitude=self._last_gnss_pos[2])

		# Calculate the bounding box for the large-scale map based on the current GNSS position and prefered map radius
		large_scale_bbox_gpd = utils.calculate_gnss_bbox((self._last_gnss_pos[0], self._last_gnss_pos[1]), self._large_map_radius)

		# Calculate the intersections between the bounding box and the watershapefile
		large_scale_map_gpd = utils.calculate_map_intersections(large_scale_bbox_gpd, self._shapefile_gpd)

		if not np.any(large_scale_map_gpd.is_empty):

			# Calculate the binary image mask from the large-scale map using the given map resolution
			image_mask = utils.calculate_mask_from_gpd(large_scale_bbox_gpd, large_scale_map_gpd, self._map_resolution)

			# Rotate the image mask based on the current yaw angle
			rotated_image_mask = utils.rotate_image_mask(image_mask, yaw_deg=self._last_yaw_angle)

			# Extract a metric map from the rotated image mask using the map resolution and large map radius
			metric_mask = utils.extract_metric_map(rotated_image_mask, self._map_resolution, self._large_map_radius, ground_coverage)

			# Scale the metric map to match the camera resolution
			scaled_mask = utils.scale_mask(metric_mask, self._camera_resolution)
		
		else:
			scaled_mask = np.zeros((self._camera_resolution[1], self._camera_resolution[0]))

		mask_img = utils.mask_to_image(scaled_mask, mask_values=[0, 255])

		# Publish the final image mask
		self._publish_mask_image(mask_img)

		# Make responce for service call
		res = sendMaskResponse()
		res.message = "Mask created!"
		res.image_data = self.bridge.cv2_to_imgmsg(mask_img, "mono8")
		return res


	def _publish_mask_image(self, mask):
		mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
		self.mask_pub.publish(mask_msg)


	def _shutdown(self):
		rospy.loginfo("Shutting down offline map Segmentation node")

	def start(self):
		rospy.loginfo("Starting offline map Segmentation node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		Segmentation_worker = Map_segmentation()
		Segmentation_worker.start()

if __name__ == "__main__":
		main()
