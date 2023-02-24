#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
import pix2geo_utils.utils 
from scipy.spatial.transform import Rotation


'''
This  node will need to have access to the following data:
	Parameters:
		- Image size: width / height
		- Camera FOV

	Topics it need to access:
		- Track detections
		- WORLD NED POS OF DRONE ( This will be delayed relative to the BBOXs -> Save last couple of messages and do approx based on header? )
		- Some sort of compass heading etc. 

	Node publishing:
		- Custom message with detection type, track ID, and coordinates?


	This node will need to do the following:
		- Extract bbx info from the track detections
		- Estimate camera coordinates for detections
		- transform the detections into "global" world frame -> transformation matrix from camera to world compensating for yaw angle?
		- Possibly also estimate GLOBAL GNSS positions of objects? Use python package for transformation

'''

from yolov8_ros.msg import BoundingBox, BoundingBoxes
from pix2geo.msg import TrackWorldCoordinate

class Pix2Geo:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("pix2geo", anonymous=False)

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
		self._last_compass_meas = None
		self._last_gnss_meas = None

		self._img_width = rospy.get_param("/drone/camera/img_width")
		self._img_height = rospy.get_param("/drone/camera/img_height")
		self._camera_hfov = rospy.get_param("/drone/camera/camera_hfov")
		self._aspect_ratio = self._img_width / self._img_height

		self._camera_fov = (self._camera_hfov, pix2geo_utils.utils.calculate_vfov(self._camera_hfov, self._aspect_ratio))

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["tracks"], 
			BoundingBoxes, 
			self._new_tracks_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["rpy"], 
			PoseStamped, 
			self._new_compass_meas_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["NED_gnss"], 
			PointStamped, 
			self._new_NED_gnss_meas_callback
		)

	def _setup_publishers(self):
		self._track_world_coord_pub = rospy.Publisher(
			self.config["topics"]["output"]["track_world_coordinate"], 
			TrackWorldCoordinate, 
			queue_size=10
		)

	def _new_tracks_callback(self, bounding_boxes):
		tracks = self._extract_bbs(bounding_boxes.bounding_boxes)

		for track in tracks:
			track_id = track[3]
			track_class = track[2]
			track_probability = track[1]
			
			center = pix2geo_utils.utils.get_bounding_box_center(track[0])

			detection_camera_frame = pix2geo_utils.utils.calculate_detection_location(
				camera_fov=self._camera_fov,
				detection_pixels=center,
				drone_position=self._last_gnss_meas,
				img_height=self._img_height,
				img_width=self._img_width
			)

			
			detection_world_frame = pix2geo_utils.utils.transform_point_cam_to_world(
				detection_camera_frame,
				translation=self._last_gnss_meas,
				yaw_deg=np.rad2deg(self._last_compass_meas)
			)
		

			self._publish_track_world_coordinate(detection_world_frame, track_id, track_probability, track_class)

	def _new_compass_meas_callback(self, measurement):
		quaternions=[
			measurement.pose.orientation.x,
			measurement.pose.orientation.y,
			measurement.pose.orientation.z,
			measurement.pose.orientation.w
		]
		
		rot = Rotation.from_quat(quaternions)
		(roll, pitch, yaw) = rot.as_euler('xyz', degrees=False)
		self._last_compass_meas = yaw

	def _new_NED_gnss_meas_callback(self, measurment):
		self._last_gnss_meas = [measurment.point.x, measurment.point.y, measurment.point.z]

	def _extract_bbs(self, bboxes):
		result = []
		for bbox in bboxes:
			left = bbox.xmin
			top = bbox.ymin
			width = bbox.xmax - bbox.xmin
			height = bbox.ymax - bbox.ymin
			result.append(([left, top, width, height], bbox.probability, bbox.Class, bbox.id))
		return result

	def _prepare_out_message(self, world_coordinates, track_id, det_probability, class_name):
		msg = TrackWorldCoordinate()
		msg.header.stamp = rospy.Time.now()

		msg.x = world_coordinates[0]
		msg.y = world_coordinates[1]
		msg.z = world_coordinates[2]
		msg.track_id = track_id
		msg.probability = det_probability
		msg.class_name = class_name

		return msg

	def _publish_track_world_coordinate(self, world_coordinates, track_id, det_probability, class_name):
		msg = self._prepare_out_message(world_coordinates, track_id, det_probability, class_name)
		self._track_world_coord_pub.publish(msg)

	def _shutdown():
		rospy.loginfo("Shutting pix2geo node")

	def start(self):
		rospy.loginfo("Starting pix2geo node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		converter = Pix2Geo()
		converter.start()

if __name__ == "__main__":
		main()
