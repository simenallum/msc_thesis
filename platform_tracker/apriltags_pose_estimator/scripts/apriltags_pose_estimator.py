#!/usr/bin/env python3

import rospy
import sensor_msgs.msg

import os
import sys
import yaml
import time
import numpy as np

from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


import apriltags_utils.apriltags_detector as apriltags_detector
import apriltags_utils.pose_recovery as pose_recovery

from anafi_uav_msgs.msg import EulerPose, Float32Stamped

class aprilTagsPoseEstimator():

		def __init__(self, config_file=None):

				rospy.init_node("aprilTags_pose_estimator", anonymous=False)

				# In order to be able to run program without ROS launch so that it can
				# be run in vscode debugger
				if config_file is None:
						config_file = rospy.get_param("~config_file")

				script_dir = os.path.dirname(os.path.realpath(__file__))

				try:
						with open(f"{script_dir}/../config/{config_file}") as f:
								self.config = yaml.safe_load(f)
				except Exception as e:
						rospy.logerr(f"Failed to load config: {e}")
						sys.exit()

				self.env = rospy.get_param("~environment")
				self.img_height = rospy.get_param("/drone/camera/img_height")
				self.img_width = rospy.get_param("/drone/camera/img_width")
				self.camera_offsets = np.array([
						rospy.get_param("/drone/camera/offset_x_mm")/1000,
						rospy.get_param("/drone/camera/offset_y_mm")/1000,
						rospy.get_param("/drone/camera/offset_z_mm")/1000
				])

				self.K = np.array(rospy.get_param("/drone/camera/camera_matrix")).reshape(3,3)
				self.focal_length = (self.K[0,0] + self.K[1,1])/2

				self.calculate_run_times = rospy.get_param("~calculate_run_times")
				self.view_camera_output = rospy.get_param("~view_camera_output")
				self.calculate_fullpipe_runtimes = rospy.get_param("~calculate_fullpipe_runtimes")

				self.latest_image = np.zeros((self.img_height, self.img_width, 3))
				self.latest_image_header = Header()
				self.new_image_available = False
				self.processing_image = False
				self.detector_initialized = False

				rospy.Subscriber("/anafi/image/downsampled", sensor_msgs.msg.Image,
						self._new_image_cb
				)

				self.pose_estimate_publisher = rospy.Publisher(
						"/estimate/aprilTags/pose", EulerPose, queue_size=10
				)

				self.pose_estimate_publisher_camera_frame = rospy.Publisher(
						"/estimate/aprilTags/pose/camera_frame", EulerPose, queue_size=10
				)

				self.num_tags_detected_publisher = rospy.Publisher(
						"/estimate/aprilTags/num_tags_detected", Float32Stamped, queue_size=10
				)

				self.feature_dists_metric = np.loadtxt(
						f"{script_dir}/../{self.config['feature_dists_metric']['path']}"
				)

				self._initialize_services()
				self.process = True

				self.detector = apriltags_detector.aprilTagDetector(self.img_width, self.img_height)
				self.pose_recoverer = pose_recovery.PoseRecovery(self.K, self.camera_offsets)

				self.full_pipe_runtimes = []

				self.detector_initialized = True

		def _initialize_services(self):

			self.srv_process = rospy.Service(
			self.config["services"]["AT_node_activation_name"],
			SetBool, 
			self._handle_process
		)
			
		def _handle_process(self, req):
			if req.data:
				self.process = True
				res = SetBoolResponse()
				res.success = True
				res.message = "Started processing data"
				return res 
			else:
				self.process = False
				res = SetBoolResponse()
				res.success = True
				res.message = "Stopped processing data"
				return res

		def run_pipeline(self, img):
				img = img.astype(np.uint8)

				if self.calculate_run_times:
						start_time = time.time()

				detections = self.detector.find_april_tag(img)

				if self.calculate_run_times:
						corner_detection_duration = time.time() - start_time
						print(f"Used {corner_detection_duration:.4f} sec to detect corners of aprilTags")

				if self.view_camera_output:
						self.detector.show_tag_corners_found(img, detections)
						
				if len(detections) == 0:
						return

				if self.calculate_run_times:
						start_time = time.time()

				features_image, feature_metric = self.detector.match_features(detections, self.feature_dists_metric)
		
				if self.calculate_run_times:
						corner_identification_duration = time.time() - start_time
						print(f"Used {corner_identification_duration:.4f} sec to prepare features")

				if self.calculate_run_times:
						start_time = time.time()

				R_cam, t_cam = self.pose_recoverer.find_R_t_pnp(feature_metric, features_image)
				R_body, t_body = self.pose_recoverer.camera_to_drone_body_frame(R_cam, t_cam)
				pose_body = self.pose_recoverer.get_pose_from_R_t(R_body, t_body)

				if self.calculate_run_times:
						pose_calculation_duration = time.time() - start_time
						print(f"Used {pose_calculation_duration:.4f} sec to calculate pose")

				if self.calculate_run_times:
						start_time = time.time()

				self._publish_num_tags(len(detections))
				self._publish_pose(pose_body)
				self._publish_camera_pose([t_cam[0], t_cam[1], t_cam[2], 0, 0, 0]) # rotation not relevant

				if self.calculate_run_times:
					publish_calculation_times = time.time() - start_time
					print(f"Used {publish_calculation_times:.4f} sec to publish pose")


		def _new_image_cb(self, msg):
				if not self.process or not self.detector_initialized:
					return

				if self.calculate_fullpipe_runtimes:
					start_time = time.time()
				
				self.run_pipeline(np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height, msg.width, -1))

				if self.calculate_fullpipe_runtimes:
						full_pipe_duration = time.time() - start_time
						self.full_pipe_runtimes.append(full_pipe_duration)
						print(f"Used {full_pipe_duration:.4f} sec to run apriltag pipe")

		def _publish_camera_pose(self, pose):
			msg = EulerPose()
			msg.header.stamp = self.latest_image_header.stamp
			msg.header.frame_id = "camera"

			msg.x = pose[0]
			msg.y = pose[1]
			msg.z = pose[2]
			msg.phi = pose[3]
			msg.theta = pose[4]
			msg.psi = pose[5]

			self.pose_estimate_publisher_camera_frame.publish(msg)


		def _publish_pose(self, pose):
				msg = EulerPose()
				msg.header.stamp = self.latest_image_header.stamp
				msg.header.frame_id = "body"

				msg.x = pose[0]
				msg.y = pose[1]
				msg.z = pose[2]
				msg.phi = pose[3]
				msg.theta = pose[4]
				msg.psi = pose[5]

				self.pose_estimate_publisher.publish(msg)

		def _publish_num_tags(self, num_tags):
				msg = Float32Stamped()
				msg.header.stamp = self.latest_image_header.stamp
				msg.header.frame_id = "NaN"

				msg.data = num_tags

				self.num_tags_detected_publisher.publish(msg)

		def _shutdown(self):
			np_list = np.array(self.full_pipe_runtimes)
			m = np_list.mean()
			std = np.std(np_list)
			median = np.median(np_list)

			print("Full pipe average runtime: ", m)
			print("Full pipe meadian runtime: ", median)
			print("Full pipe frequency: ", 1/m)
			print("Full pipe std. dev: ", std)

		def start(self):
				rospy.loginfo("Starting AprilTags pose estimator")

				rospy.on_shutdown(self._shutdown)

				while not rospy.is_shutdown():

					rospy.spin()


def main():
		estimator = aprilTagsPoseEstimator(config_file="apriltags_config.yaml")
		estimator.start()

if __name__ == "__main__":
		main()