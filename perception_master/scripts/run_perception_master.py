#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
import folium
from folium.plugins import MarkerCluster
import pandas as pd

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Vector3Stamped
from pix2geo.msg import TrackWorldCoordinate
from perception_master.msg import DetectedPerson
from std_srvs.srv import SetBool, SetBoolRequest

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
		self.all_proxies_loaded = False
		self._tracking_platform = False

		# Platform detection utils
		self._min_dist_to_initiate_platform_tracking = self.config["settings"]["min_dist_to_initiate_platform_tracking"]

		# Human and FO detection
		self._radius_of_acceptance_new_human_detections = self.config["settings"]["radius_of_acceptance_new_human_detections"]
		self._radius_of_acceptance_new_FO_detections = self.config["settings"]["radius_of_acceptance_new_FO_detections"]
		self._critical_level_distances = self.config["settings"]["critical_level_distances"]
		self._track_confidence_threshold = self.config["settings"]["track_confidence_threshold"]
		self._human_table = {}
		self._FO_table = {}
		self._track_critical_level_table = {}

		# Safe point generation
		self._radius_of_acceptance_new_safe_points = self.config["settings"]["radius_of_acceptance_new_safe_points"]
		self._safe_points_list = []

		# GNSS convertion
		self._NED_frame_origin = [None, None, None]
		self._gnss_drone_coordinates = []
		self._GNSS_safe_points_tempfile = self.config["temp_filepaths"]["safe_points"]
		self._GNSS_humans_tempfile = self.config["temp_filepaths"]["human_points"]
		self._GNSS_FO_tempfile = self.config["temp_filepaths"]["FO_points"]

		self._timer = rospy.Timer(rospy.Duration(self.config["settings"]["GNSS_saving_interval"]), self._save_points_as_gnss_timer_callback)


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
			PoseWithCovarianceStamped, 
			self._new_platform_EKF_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["GNSS"], 
			NavSatFix, 
			self._new_GNSS_measurement_callback
		)

		rospy.Subscriber(
			self.config["topics"]["input"]["ned_frame_origin"], 
			Vector3Stamped, 
			self._new_gnss_origin_callback
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

			self.all_proxies_loaded = True
		except:
			rospy.logerr(f"[Perception master] Couldt not detect one of the activation nodes ROS services -- > Exiting")
			rospy.signal_shutdown("Service not detected!")

		self._activate_AT_node_proxy = rospy.ServiceProxy(self._AT_node_activation_name, SetBool)
		self._activate_DNN_node_proxy = rospy.ServiceProxy(self._DNN_node_activation_name, SetBool)
		self._activate_GNSS_node_proxy = rospy.ServiceProxy(self._GNSS_node_activation_name, SetBool)

	def _save_points_as_gnss_timer_callback(self, event):
		
		if len(self._safe_points_list) > 0:
			safe_points_gnss = utils.convert_ned_list_to_gnss_list(self._NED_frame_origin, self._safe_points_list)
			utils.save_np_list_as_csv(self._GNSS_safe_points_tempfile, np.array(safe_points_gnss))

		human_points_ned_list = utils.get_dict_values_as_list(self._human_table)
		if len(human_points_ned_list) > 0:
			human_points_gnss = utils.convert_ned_list_to_gnss_list(self._NED_frame_origin, human_points_ned_list)
			utils.save_np_list_as_csv(self._GNSS_humans_tempfile, np.array(human_points_gnss))

		FO_points_ned_list = utils.get_dict_values_as_list(self._FO_table)
		if len(FO_points_ned_list) > 0:
			FO_points_gnss = utils.convert_ned_list_to_gnss_list(self._NED_frame_origin, FO_points_ned_list)
			utils.save_np_list_as_csv(self._GNSS_FO_tempfile, np.array(FO_points_gnss))

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

	def _new_GNSS_measurement_callback(self, GNSS_meas):
		self._gnss_drone_coordinates.append([GNSS_meas.latitude, GNSS_meas.longitude, GNSS_meas.altitude])
		
	def _new_gnss_origin_callback(self, origin_msg):
		if None in (self._NED_frame_origin):
			self._NED_frame_origin = [origin_msg.vector.x, origin_msg.vector.y, origin_msg.vector.z]

	def _new_safe_point_callback(self, safe_point_msg):

		new_safe_point = self._extract_point_msg(safe_point_msg)
		point_in_table = utils.is_point_within_threshold_list(
								new_safe_point, 
							   	self._radius_of_acceptance_new_safe_points,
								self._safe_points_list)
		
		if not point_in_table:
			self._safe_points_list.append(new_safe_point)

			self._publish_safe_point(new_safe_point, safe_point_msg.header)

	def _new_track_callback(self, track_msg):
		track = self._extract_track_msg(track_msg)

		# Only handle track if confidence is high enough
		if track["confidence"] <= self._track_confidence_threshold:
			return

		# Handle human tracks
		if track["class_name"] == "human":
			pub_track = False

			# Track_id already exists
			if utils.key_exists(track["track_id"], self._human_table):

				# Check if distance is larger than the threshold for accepting new tracks
				distance_between_measurements = utils.calculate_euclidian_distance(track["point"], self._human_table[track["track_id"]])
				if distance_between_measurements >= self._radius_of_acceptance_new_human_detections:
					# Update position for track
					self._human_table[track["track_id"]] = track["point"]

					pub_track = True

			# Track ID does not exist
			else:
				# Check if distance is larger than the threshold for accepting new tracks
				point_in_table = utils.is_point_within_threshold_dict(track["point"], self._radius_of_acceptance_new_human_detections, self._human_table)
				
				if not point_in_table:
					# Add new track and point
					self._human_table[track["track_id"]] = track["point"]

					pub_track = True


			if pub_track: 
				# Publish the track and safe severity level in another table.
				distance_to_FO = utils.get_closest_distance(track["point"], self._FO_table)
				critical_level = utils.calculate_critical_level(distance_to_FO, self._critical_level_distances)
				self._track_critical_level_table[track["track_id"]] = critical_level
				
				self._publish_humans_detection(track["point"], critical_level, track["track_id"], track_msg.header)
					

		# Handle FO tracks
		elif track["class_name"] == "floating-objects":
			# Track_id already exists
			if utils.key_exists(track["track_id"], self._FO_table):
				distance_between_measurements = utils.calculate_euclidian_distance(track["point"], self._FO_table[track["track_id"]])
				if distance_between_measurements >= self._radius_of_acceptance_new_FO_detections:
					self._FO_table["track_id"] = track["point"]

			else:
				point_in_table = utils.is_point_within_threshold_dict(track["point"], self._radius_of_acceptance_new_FO_detections, self._FO_table)
				if not point_in_table:
					self._FO_table[track["track_id"]] = track["point"]


			# iterate trough human table and re-publish tracks with changed severity level
			for key in self._human_table:
				distance_to_FO = utils.calculate_euclidian_distance(self._human_table[key], self._FO_table[track["track_id"]])
				critical_level = utils.calculate_critical_level(distance_to_FO, self._critical_level_distances)

				if critical_level != self._track_critical_level_table[key]:
					self._track_critical_level_table[key] = critical_level

					self._publish_humans_detection(self._human_table[key], critical_level, key, track_msg.header)



	def _new_platform_EKF_callback(self, ekf_msg):
		if not self.all_proxies_loaded:
			return
		
		vector = self._extract_ekf_msg(ekf_msg)

		length = utils.calculate_euclidian_distance_of_vector(vector)

		if length > self._min_dist_to_initiate_platform_tracking:
			if self._tracking_platform:
				rospy.loginfo("Stopped tracking using perception based estimators. Activated GNSS.")

				# create a request object
				request = SetBoolRequest()
				request.data = True

				response = self._activate_GNSS_node_proxy(request)

				request = SetBoolRequest()
				request.data = False

				response = self._activate_AT_node_proxy(request)
				response = self._activate_DNN_node_proxy(request)
				
				self._tracking_platform = False

		else:
			if not self._tracking_platform:
				rospy.loginfo("Initiated tracking using perception based estimators. Deactivated GNSS.")
				
				# create a request object
				request = SetBoolRequest()
				request.data = True

				response = self._activate_AT_node_proxy(request)
				response = self._activate_DNN_node_proxy(request)

				request = SetBoolRequest()
				request.data = False

				response = self._activate_GNSS_node_proxy(request)

				self._tracking_platform = True

			


	def _extract_point_msg(self, point_msg):
		return np.array([point_msg.point.x, point_msg.point.y, point_msg.point.z])
	
	def _extract_track_msg(self, track_msg):
		content = {
			"point": np.array([track_msg.x, track_msg.y, track_msg.z]),
			"track_id": track_msg.track_id,
			"class_name": track_msg.class_name,
			"confidence": track_msg.probability
		}

		return content
	
	def _extract_ekf_msg(self, ekf_msg):
		return np.array([ekf_msg.pose.pose.position.x, ekf_msg.pose.pose.position.y, ekf_msg.pose.pose.position.z])
	
	def _publish_safe_point(self, safe_point, header):
		safe_point_message = PointStamped()
		safe_point_message.header = header
		safe_point_message.point.x = safe_point[0]
		safe_point_message.point.y = safe_point[1]
		safe_point_message.point.z = safe_point[2]

		self._safe_point_pub.publish(safe_point_message)

	def _publish_humans_detection(self, human_point, critical_level, track_id, header):
		human_detection_msg = DetectedPerson()
		human_detection_msg.header = header
		human_detection_msg.position.x = human_point[0]
		human_detection_msg.position.y = human_point[1]
		human_detection_msg.position.z = human_point[2]
		human_detection_msg.severity = critical_level
		human_detection_msg.id = track_id

		self._detected_person_pub.publish(human_detection_msg)

	def _shutdown(self):
		rospy.loginfo("[Perception master]: Shutting down node")

		# Create a folium map centered on the first NED frame origin
		map = folium.Map(location=[self._NED_frame_origin[0], self._NED_frame_origin[1]], zoom_start=10)

		# Create a marker cluster for the safe points
		safe_points_cluster = folium.FeatureGroup(name='Safe points').add_to(map)

		# Add circle markers for each safe point to the feature group
		safe_points = pd.read_csv(self._GNSS_safe_points_tempfile, names=['LATITUDE', 'LONGITUDE', 'ALTITUDE'], sep=',')
		for lat, lon in zip(safe_points['LATITUDE'], safe_points['LONGITUDE']):
			folium.CircleMarker(location=[lat, lon], radius=2, color='red', fill_color='red', fill_opacity=0.8).add_to(safe_points_cluster)

		# Create a marker cluster for the FO points
		FO_points_cluster = folium.FeatureGroup(name='FO points').add_to(map)

		# Add circle markers for each FO point to the feature group
		FO_points = pd.read_csv(self._GNSS_FO_tempfile, names=['LATITUDE', 'LONGITUDE', 'ALTITUDE'], sep=',')
		for lat, lon in zip(FO_points['LATITUDE'], FO_points['LONGITUDE']):
			folium.CircleMarker(location=[lat, lon], radius=2, color='blue', fill_color='blue', fill_opacity=0.8).add_to(FO_points_cluster)

		# Create a marker cluster for the human points
		human_points_cluster = folium.FeatureGroup(name='Human points').add_to(map)

		# Add circle markers for each human point to the feature group
		human_points = pd.read_csv(self._GNSS_humans_tempfile, names=['LATITUDE', 'LONGITUDE', 'ALTITUDE'], sep=',')
		for lat, lon in zip(human_points['LATITUDE'], human_points['LONGITUDE']):
			folium.CircleMarker(location=[lat, lon], radius=2, color='green', fill_color='green', fill_opacity=0.8).add_to(human_points_cluster)


		# Create a feature group for the drone trajectory
		drone_polyline_group = folium.FeatureGroup(name='Drone Trajectory').add_to(map)
		drone_loc_np = np.array(self._gnss_drone_coordinates)
		drone_polyline = folium.PolyLine(locations=drone_loc_np[:,:2], color='orange', weight=2, opacity=0.7)
		drone_polyline.add_to(drone_polyline_group)

		# Add a legend to the map
		folium.LayerControl().add_to(map)

		# Save the map as an HTML file
		map.save(self.config["temp_filepaths"]["map_filepath"])

		rospy.loginfo("[Perception master] Map saved!")


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
