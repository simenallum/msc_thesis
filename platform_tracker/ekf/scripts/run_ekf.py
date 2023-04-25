#!/usr/bin/env python3

import os
import sys
import yaml
import time

import rospy
import tf2_ros as tf2
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse



import geometry_msgs.msg
from anafi_uav_msgs.msg import EulerPose, Float32Stamped
import numpy as np
from geometry_msgs.msg import Vector3Stamped, TwistStamped, PointStamped, PoseWithCovarianceStamped

from ekf_utils.ekf import EKF, EKFState
import dynamic_models.dynamic_models as dynamic_models
import measurement_models.measurement_models as measurement_models

class EKFRosRunner():

		def __init__(self):

						
				rospy.init_node("perception_ekf", anonymous=False)

				# Load config
				config_file = rospy.get_param("~config_file")
				script_dir = os.path.dirname(os.path.realpath(__file__))

				try:
						with open(f"{script_dir}/../config/{config_file}") as f:
								self.config = yaml.safe_load(f)
				except Exception as e:
						rospy.logerr(f"Failed to load config: {e}")
						sys.exit()

				# Get dynamic models based on config file
				self.dynamic_model_type = self.config["ekf"]["dynamic_model"]
				self.dynamic_model = dynamic_models.get_dynamic_model_from_type(
						self.dynamic_model_type,
						self.config["dynamic_models"][self.dynamic_model_type]["sigmas"]
				)

				# Get measurement models based on config file
				self.measurement_model_types = self.config["ekf"]["measurement_models"]
				self.measurement_models_dict = measurement_models.get_measurement_models_from_types(
						self.measurement_model_types, self.config["measurements"]
				)

				rospy.loginfo(f"EKF dynamic model: {self.dynamic_model_type}")
				rospy.loginfo(f"EKF measurement model(s): {self.measurement_model_types}")

				# Create estimate publisher based on the states used in the dynamic model
				self.output_states = self.config["dynamic_models"][self.dynamic_model_type]["output_states"]
				self.estimate_publisher = self._get_estimate_publisher(self.output_states)

				self.velocity_pub = rospy.Publisher(
						"/estimate/ekf/velocity", Vector3Stamped, queue_size=10
				)

				# Create ROS subscribers for the input and measurements defined in the config file
				self.has_inputs = True
				self._prev_attitude: np.ndarray = None
				self._setup_subscribers()

				self.initialize_services()
				self.use_gnss_measurement = False

				# Set up filter
				self.filter = EKF(self.dynamic_model, self.measurement_models_dict)

				self.x0 = np.array(self.config["dynamic_models"][self.dynamic_model_type]["init_values"]["x0"])
				self.P0 = np.diag(np.array(self.config["dynamic_models"][self.dynamic_model_type]["init_values"]["P0"]))**2

				self.ekf_estimate = EKFState(self.x0, self.P0)

				self.dt = self.config["ekf"]["dt"]
				self.rate = rospy.Rate(1/self.dt)

				## The altitude measurement is unusable in the startup-phase and has to be delayed started.
				# self.timer = rospy.Timer(rospy.Duration(20), self._trigger_altitude_measurements_cb, oneshot=True)
				self.altitude_enabled = False

				# Keep track of when the first measurement arrives and only then start
				# the filter
				self.estimating = False

				self.tfBuffer = tf2.Buffer()
				self.listener = tf2.TransformListener(self.tfBuffer)

				self.iter_time = []

				self.start_time = rospy.get_rostime()

		def initialize_services(self):
			self.srv_use_gnss_data = rospy.Service(
			self.config["services"]["GNSS_node_activation_name"],
			SetBool, 
			self._handle_use_gnss_data
		)
			
		def _handle_use_gnss_data(self, req):
			if req.data:
				self.use_gnss_measurement = True
				res = SetBoolResponse()
				res.success = True
				res.message = "Started using GNSS data"
				return res 
			else:
				self.use_gnss_measurement = False
				res = SetBoolResponse()
				res.success = True
				res.message = "Stopped using GNSS data"
				return res

		def _shutdown(self):
			np_list = np.array(self.iter_time)
			m = np_list.mean()
			std = np.std(np_list)
			median = np.median(np_list)
			max = np.max(np_list)

			print("Pipe average runtime: ", m)
			print("Pipe meadian runtime: ", median)
			print("Pipe frequency: ", 1/m)
			print("Pipe std. dev: ", std)
			print("Pipe max runtime: ", max)

		def run(self):
				rospy.loginfo("Starting perception EKF")

				rospy.on_shutdown(self._shutdown)

				# If there are inputs coming into the system, the precition step will be
				# performed each time a new input arrives and this is handled in the respective
				# input callback
				if self.has_inputs:
						rospy.spin()
				else:
						while not rospy.is_shutdown():

								if self.estimating:
										start_time = time.time()

										self.ekf_estimate = self.filter.predict(self.ekf_estimate, None, self.dt)
										output_msg = self._pack_estimate_msg(self.ekf_estimate, self.output_states)
										self.estimate_publisher.publish(output_msg)
										self.publish_velocity_states()

										runtime = time.time() - start_time
										self.iter_time.append(runtime)

								self.rate.sleep()
								
		def _trigger_altitude_measurements_cb(self, timer):
			self.altitude_enabled = True

		def _get_estimate_publisher(self, output_states: str):
				if output_states == "position":
						publisher = rospy.Publisher(
								self.config["ekf"]["output"]["topic_name"],
								PoseWithCovarianceStamped, queue_size=10
						)
				else:
						raise NotImplementedError

				return publisher

		def _setup_subscribers(self):

				rospy.loginfo(f"Entered Subscribers")

				input_config = self.config["dynamic_models"][self.dynamic_model_type]["input"]

				if input_config["type"] == "control_inputs":
						raise NotImplementedError
				elif input_config["type"] == "none":
						self.has_inputs = False
				else:
						raise NotImplementedError

				for mm in self.measurement_model_types:
						if mm == "dnn_cv_position":
								rospy.Subscriber(
										self.config["measurements"]["dnn_cv_position"]["topic_name"],
										geometry_msgs.msg.PointStamped, self._dnn_cv_position_cb
								)
						elif mm == "drone_velocity":
								rospy.Subscriber(
										self.config["measurements"]["drone_velocity"]["topic_name"],
										TwistStamped, self._drone_velocity_cb
								)
						elif mm == "apriltags_position":
								rospy.Subscriber(
										self.config["measurements"]["apriltags_position"]["topic_name"],
										EulerPose, self._apriltags_position_cb
								)
						elif mm == "altitude_measurement":
								rospy.Subscriber(
										self.config["measurements"]["altitude_measurement"]["topic_name"],
										Float32Stamped, self._altitude_measurement_cb
								)
						elif mm == "gnss_measurement":
								rospy.Subscriber(
										self.config["measurements"]["gnss_measurement"]["topic_name"],
										PointStamped, self._gnss_measurement_cb
								)
						elif mm == "dnn_cv_position_xy":
								pass # Uses same callback as dnn_cv_position
						else:
								print(f"Measurement model: {mm} not implemented")
								raise NotImplementedError


		def _pack_estimate_msg(self, ekfstate: EKFState, states_type: str):

			if states_type == "position":
					msg = PoseWithCovarianceStamped()
					
					msg.header.stamp = rospy.Time.now()
					msg.header.frame_id = "body"
					msg.pose.pose.position.x = ekfstate.mean[0]
					msg.pose.pose.position.y = ekfstate.mean[1]
					msg.pose.pose.position.z = ekfstate.mean[2]

					cov = np.eye(6)
					cov[:3,:3] = ekfstate.cov[:3,:3]

					msg.pose.covariance = cov.flatten().tolist()
			else:
					raise NotImplementedError

			return msg

		def publish_velocity_states(self):
			vel = Vector3Stamped()

			vel.header.stamp = rospy.Time.now()
			vel.header.frame_id = "body"

			vel.vector.x = self.ekf_estimate.mean[3]
			vel.vector.y = self.ekf_estimate.mean[4]
			vel.vector.z = self.ekf_estimate.mean[5]

			self.velocity_pub.publish(vel)

		def _altitude_measurement_cb(self, msg: Float32Stamped):
				# Only use internal height measurement if we are not in takeoff phase and if below a certain value
				if self.altitude_enabled and self.ekf_estimate.mean[2] < 1.0: #TODO Remove magic constant
						z = np.array([msg.data])
						self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "altitude_measurement")
		
		def _gnss_measurement_cb(self, msg: PointStamped):
			if not self.use_gnss_measurement:
					return
			
			z = np.array([msg.point.x, msg.point.y, msg.point.z])
			self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "gnss_measurement")

		def _dnn_cv_position_cb(self, msg: geometry_msgs.msg.PointStamped):
				# Only use dnncv for altitude measurement if above certain altitude (not reliable below)
				if self.ekf_estimate.mean[2] > 1.0: #TODO Remove magic constant
						z = np.array([msg.point.x, msg.point.y, msg.point.z])
						self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "dnn_cv_position")
				else:
						z = np.array([msg.point.x, msg.point.y])
						self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "dnn_cv_position_xy")

		def _apriltags_position_cb(self, msg: EulerPose):
			# Turn on the altitude at first seen AT
			if not self.altitude_enabled:
				self.altitude_enabled = True

			# Turn on the filter at first seen AT -> aka take-off
			if not self.estimating:
				self.estimating = True
				
			z = np.array([msg.x, msg.y, msg.z])

			self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "apriltags_position")

		def _drone_velocity_cb(self, msg: TwistStamped):
				z = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

				# # Remove entries that are below a certain threshold as these are not accurate
				z[np.where(np.abs(z) < 0.01)] = 0

				self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "drone_velocity")

def main():
		ekf_runner = EKFRosRunner()
		ekf_runner.run()

if __name__ == "__main__":
		main()