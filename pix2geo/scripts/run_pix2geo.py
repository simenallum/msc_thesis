#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import pix2geo_utils.utils 
from scipy.spatial.transform import Rotation

from yolov8_ros.msg import BoundingBox, BoundingBoxes
from pix2geo.msg import TrackWorldCoordinate
from anafi_uav_msgs.msg import Float32Stamped
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
		self._last_compass_meas = [None]
		self._last_gnss_meas = [None]
		self._last_height_meas = [None]

		self._flag_publish_camera_frame_detections = self.config["flags"]["publish_camera_coordinates"]

		self._img_width = rospy.get_param("/drone/camera/img_width")
		self._img_height = rospy.get_param("/drone/camera/img_height")
		self._camera_hfov = rospy.get_param("/drone/camera/camera_hfov")
		self._aspect_ratio = self._img_width / self._img_height
		self.camera_matrix = np.array(rospy.get_param("/drone/camera/camera_matrix")).reshape(3,3)
		self._camera_focal_length = (self.camera_matrix[0,0] + self.camera_matrix[1,1])/2
		self._image_center = (self.camera_matrix[0,2], self.camera_matrix[1,2])
		self._camera_fov = (self._camera_hfov, pix2geo_utils.utils.calculate_vfov(self._camera_hfov, self._aspect_ratio))

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["tracks"], 
			BoundingBoxes, 
			self._new_tracks_callback
		)
		
		rospy.Subscriber(
			self.config["topics"]["input"]["safe_points_cam_frame"], 
			PointStamped, 
			self._new_safe_points_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["pose"], 
			PoseStamped, 
			self._new_compass_meas_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["NED_gnss"], 
			PointStamped, 
			self._new_NED_gnss_meas_callback
		)

		rospy.Subscriber(
			"/anafi/height",
			Float32Stamped,
			self._new_height_CB
		)

	def _setup_publishers(self):
		self._track_world_coord_pub = rospy.Publisher(
			self.config["topics"]["output"]["track_world_coordinate"], 
			TrackWorldCoordinate, 
			queue_size=10
		)

		self._safe_point_world_coord_pub = rospy.Publisher(
			self.config["topics"]["output"]["safe_point_world_coordinate"], 
			PointStamped, 
			queue_size=10
		)

		if self._flag_publish_camera_frame_detections:
			self._track_camera_coord_pub_triangulate = rospy.Publisher(
				self.config["topics"]["output"]["track_camera_coordinate_triangulate"], 
				TrackWorldCoordinate, 
				queue_size=10
			)

			self._track_camera_coord_pub_FOV = rospy.Publisher(
				self.config["topics"]["output"]["track_camera_coordinate_fov"], 
				TrackWorldCoordinate, 
				queue_size=10
			)

	def _new_tracks_callback(self, bounding_boxes):
		if None in (self._last_height_meas + self._last_gnss_meas + self._last_compass_meas):
			rospy.logwarn(f"Can not transform safe point. Missing either compass or gnss-measurements")
			return
		
		tracks = self._extract_bbs(bounding_boxes.bounding_boxes)

		for track in tracks:
			track_id = track[3]
			track_class = track[2]
			track_probability = track[1]
			
			center = pix2geo_utils.utils.get_bounding_box_center(track[0])

			detection_camera_frame_fov = pix2geo_utils.utils.calculate_detection_location(
				camera_fov=self._camera_fov,
				detection_pixels=center,
				drone_position=self._last_height_meas,
				img_height=self._img_height,
				img_width=self._img_width
			)

			detection_camera_frame_tria = pix2geo_utils.utils._pixel_to_camera_coordinates(
				center_px=center,
				drone_pos=self._last_height_meas,
				camera_focal_length=self._camera_focal_length,
				image_center=self._image_center
			)
			
			if self._flag_publish_camera_frame_detections:
				self._publish_track_camera_coordinate(detection_camera_frame_fov, detection_camera_frame_tria, track_id, track_probability, track_class)

			
			detection_world_frame = pix2geo_utils.utils.transform_point_cam_to_world(
				detection_camera_frame_tria,
				translation=self._last_gnss_meas,
				yaw_deg=np.rad2deg(self._last_compass_meas)
			)
		

			self._publish_track_world_coordinate(detection_world_frame, track_id, track_probability, track_class)

	def _new_safe_points_callback(self, point_msg):
		if None in (self._last_height_meas + self._last_gnss_meas + self._last_compass_meas):
			rospy.logwarn(f"Can not transform safe point. Missing either compass or gnss-measurements")
			return
		
		
		cam_x = point_msg.point.x
		cam_y = point_msg.point.y
			
		center = (cam_x, cam_y)

		safe_point_camera_frame_tria = pix2geo_utils.utils._pixel_to_camera_coordinates(
			center_px=center,
			drone_pos=self._last_height_meas,
			camera_focal_length=self._camera_focal_length,
			image_center=self._image_center
		)

		safe_point_world_frame = pix2geo_utils.utils.transform_point_cam_to_world(
			safe_point_camera_frame_tria,
			translation=self._last_gnss_meas,
			yaw_deg=np.rad2deg(self._last_compass_meas)
		)
		

		self._publish_safe_point_world_coordinate(safe_point_world_frame)

	def _prepare_safe_point_message(self, point):
		msg = PointStamped()
		msg.header.stamp = rospy.Time.now()

		msg.point.x = point[0]
		msg.point.y = point[1]
		msg.point.z = point[2]

		return msg

	def _publish_safe_point_world_coordinate(self, point):
		msg = self._prepare_safe_point_message(point)
		self._safe_point_world_coord_pub.publish(msg)

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

	def _new_height_CB(self, meas):
		self._last_height_meas = [meas.data]

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

	def _publish_track_camera_coordinate(self, camera_coordinates_fov, camera_coordinates_tria, track_id, det_probability, class_name):
		msg = self._prepare_out_message(camera_coordinates_fov, track_id, det_probability, class_name)
		self._track_camera_coord_pub_FOV.publish(msg)

		msg = self._prepare_out_message(camera_coordinates_tria, track_id, det_probability, class_name)
		self._track_camera_coord_pub_triangulate.publish(msg)


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
