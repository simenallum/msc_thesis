#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge


from deepsort_realtime.deepsort_tracker import DeepSort
from yolov8_ros.msg import BoundingBox, BoundingBoxes
from deepsort_tracker_utils.bounding_box_utils import convert_bboxes, draw_detections
import sensor_msgs.msg

class DeepSortTracker:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("DeepSortTracker", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file or deepsort_params is None:
			config_file = rospy.get_param("~config_file")
			deepsort_params = rospy.get_param("~deepsort_params")

		try:
			with open(f"{script_dir}/../config/{deepsort_params}") as f:
				self.deepsort_params = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		try:
			with open(f"{script_dir}/../config/{config_file}") as f:
				self.config = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		self._initalize_parameters()
		self._setup_publishers()
		self._setup_subscribers()
		self._initialize_tracker()

		self.avg_est_time = []

	def _initalize_parameters(self):
		self.bridge = CvBridge()

		self.flag_publish_images_with_tracks = self.config["settings"]["publish_image_with_tracks"]

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["bounding_boxes"], 
			BoundingBoxes, 
			self._new_bb_calback
		)

	def _setup_publishers(self):

		self.tracks_pub = rospy.Publisher(
			self.config["topics"]["output"]["tracks"], 
			BoundingBoxes, 
			queue_size=10
		)

		if self.flag_publish_images_with_tracks:
			self.image_with_tracks_pub = rospy.Publisher(
				self.config["topics"]["output"]["image_with_tracks"], 
				sensor_msgs.msg.Image, 
				queue_size=10
			)

	def _initialize_tracker(self):
		self.tracker = DeepSort(
			max_age=self.deepsort_params["max_age"],
			n_init=self.deepsort_params["n_init"],
			nms_max_overlap=self.deepsort_params["nms_max_overlap"],
			max_iou_distance=self.deepsort_params["max_iou_distance"],
			max_cosine_distance=self.deepsort_params["max_cosine_distance"],
			nn_budget=self.deepsort_params["nn_budget"],
			override_track_class=self.deepsort_params["override_track_class"],
			embedder=self.deepsort_params["embedder"],
			gating_only_position=self.deepsort_params["gating_only_position"],
			half=self.deepsort_params["half"],
			bgr=self.deepsort_params["bgr"],
			embedder_gpu=self.deepsort_params["embedder_gpu"],
			embedder_model_name=self.deepsort_params["embedder_model_name"],
			embedder_wts=self.deepsort_params["embedder_wts"],
			polygon=self.deepsort_params["polygon"],
			today=self.deepsort_params["today"],
			_lambda =self.deepsort_params["_lambda"],
			EWMA_alpha=self.deepsort_params["EWMA_alpha"],
			std_pos=self.deepsort_params["std_pos"],
			std_vel=self.deepsort_params["std_vel"],
		)

	def _new_bb_calback(self, bounding_boxes):
		clock = False
		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)
		frame = self.bridge.imgmsg_to_cv2(bounding_boxes.frame, "bgr8")

		if not len(bbs) == 0:
			start = time.time()
			clock = True
		tracks = self.tracker.update_tracks(bbs, frame=frame)
		if clock:
			end = time.time()
			ms = (end - start) * 1000
			self.avg_est_time.append(ms)
			rospy.logdebug("Time taken: {:.2f} ms".format(ms))

		track_list = self._extract_bbs_and_trackid(tracks)
		
		if self.flag_publish_images_with_tracks:
			image = draw_detections(frame, track_list)
			self._publish_image_with_tracks(image)

		if len(track_list) > 0:
			tracks_msg = self._prepare_tracks_msg(track_list)
			self._publish_confirmed_tracks(tracks_msg)

		

	def _publish_image_with_tracks(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		self.image_with_tracks_pub.publish(msg)

	def _extract_bbs(self, bounding_boxes):
		return convert_bboxes(bounding_boxes)

	def _extract_bbs_and_trackid(self, tracks):
		result = []

		for track in tracks:
			# Only publish tracks that are confirmed. State == 2 for confirmed tracks
			if track.state == 2 and track.time_since_update < 5:
				bb = track.to_tlbr(orig=False)
				conf = track.get_det_conf() or 0
				id = int(track.track_id)
				class_name = track.det_class

				result.append([bb, conf, id, class_name])

		return result

	def _publish_confirmed_tracks(self, tracks_msg):
		self.tracks_pub.publish(tracks_msg)

	def _prepare_tracks_msg(self, track_list):
		boundingBoxes = BoundingBoxes()
		boundingBoxes.header.stamp = rospy.Time.now()

		for i in range(len(track_list)):
			boundingBox = BoundingBox()
			boundingBox.xmin = int(track_list[i][0][0])
			boundingBox.ymin = int(track_list[i][0][1])
			boundingBox.xmax = int(track_list[i][0][2])
			boundingBox.ymax = int(track_list[i][0][3])
			boundingBox.probability = track_list[i][1]
			boundingBox.id = track_list[i][2]
			boundingBox.Class = track_list[i][3]

			boundingBoxes.bounding_boxes.append(boundingBox)

		return boundingBoxes

	def _shutdown(self):
		np_est = np.array(self.avg_est_time)
		rospy.loginfo(f"Average estimation runtime: {np.mean(np_est)} ms")
		rospy.loginfo(f"Variance in estimation runtime: {np.var(np_est)} ms")
		rospy.loginfo(f"St. dev in estimation runtime: {np.std(np_est)} ms")

		np.save("/home/msccomputer/catkin_ws/src/msc_thesis/deepsort_tracker/data/runtime.npy", np_est)

		rospy.loginfo("Shutting down tracker node")

	def start(self):
		rospy.loginfo("Starting tracker node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		Detector = DeepSortTracker()
		Detector.start()

if __name__ == "__main__":
		main()
