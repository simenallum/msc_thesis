#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
from cv_bridge import CvBridge

from yolov8_ros.msg import BoundingBox, BoundingBoxes
from sort_tracker_utils.bounding_box_utils import convert_bboxes, draw_detections
from sort.sort import *
import sensor_msgs.msg

class SortTracker:

	def __init__(self, config_file=None, sort_params=None):

		rospy.init_node("SortTracker", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file or sort_params is None:
			config_file = rospy.get_param("~config_file")
			sort_params = rospy.get_param("~sort_params")

		try:
			with open(f"{script_dir}/../config/{sort_params}") as f:
				self.sort_params = yaml.safe_load(f)
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

	def _initalize_parameters(self):
		self.bridge = CvBridge()
		self.class_combinations = {}

		self.flag_publish_images_with_tracks = self.config["settings"]["publish_image_with_tracks"]
		
		self.max_age = self.sort_params['max_age']
		self.min_hits = self.sort_params['min_hits']
		self.iou_threshold = self.sort_params['iou_threshold']
		self.EWMA_alpha = self.sort_params['EWMA_alpha']
		self.init_cov = self.sort_params['init_cov']
		self.measurement_cov = self.sort_params['measurement_cov']
		self.system_cov = self.sort_params['system_cov']

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
		self.tracker = Sort(self.init_cov, self.measurement_cov, self.system_cov, max_age=self.max_age, min_hits=self.min_hits)

	def _new_bb_calback(self, bounding_boxes):
		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)
		frame = self.bridge.imgmsg_to_cv2(bounding_boxes.frame, "bgr8")

		if len(bbs) == 0:
			bbs = np.empty((0, 5))
		tracks = self.tracker.update(bbs)

		track_list = self._extract_bbs_and_trackid(tracks)

		if self.flag_publish_images_with_tracks:
			image = draw_detections(frame, tracks)
			self._publish_image_with_tracks(image)

		if len(track_list) > 0:
			tracks_msg = self._prepare_tracks_msg(track_list)
			self._publish_confirmed_tracks(tracks_msg)

	def _publish_image_with_tracks(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		self.image_with_tracks_pub.publish(msg)

	def _extract_bbs(self, bounding_boxes):
		for t in bounding_boxes:
			self._add_class_combination(class_id=t.id, class_name=t.Class)

		return convert_bboxes(bounding_boxes)

	def _extract_bbs_and_trackid(self, tracks):
		result = []

		for track in tracks:
				bb = track[0:4]
				conf = track[6]
				id = track[4]
				class_name = self.class_combinations[track[5]]

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

	def _add_class_combination(self, class_id, class_name):
		self.class_combinations[int(class_id)] = class_name


	def _shutdown():
		rospy.loginfo("Shutting down tracker node")

	def start(self):
		rospy.loginfo("Starting tracker node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		Detector = SortTracker()
		Detector.start()

if __name__ == "__main__":
		main()
