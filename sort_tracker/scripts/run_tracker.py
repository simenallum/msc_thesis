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
		self.tracker = Sort()

	def _new_bb_calback(self, bounding_boxes):
		start = time.time()
		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)
		frame = self.bridge.imgmsg_to_cv2(bounding_boxes.frame, "bgr8")

		tracks = self.tracker.update(bbs)

		track_list = self._extract_bbs_and_trackid(tracks)

		print(track_list)
		
		# if self.flag_publish_images_with_tracks:
		# 	image = draw_detections(frame, track_list)
		# 	self._publish_image_with_tracks(image)

		# if len(track_list) > 0:
		# 	tracks_msg = self._prepare_tracks_msg(track_list)
		# 	self._publish_confirmed_tracks(tracks_msg)

		end = time.time()
		print("Time taken: {:.2f} ms".format((end - start) * 1000))

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

	def _shutdown():
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
