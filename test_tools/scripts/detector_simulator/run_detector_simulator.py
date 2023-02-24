#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from yolov8_ros.msg import BoundingBox, BoundingBoxes

import utils.apriltags_detector as apriltags_detector
from utils.visualiations import draw_detections




class DetectorSimulator:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("detection_simulator", anonymous=False)

		script_dir = os.path.dirname(os.path.realpath(__file__))

		if config_file or deepsort_params is None:
			config_file = rospy.get_param("~config_file")

		try:
			with open(f"{script_dir}/../../config/{config_file}") as f:
				self.config = yaml.safe_load(f)
		except Exception as e:
				rospy.logerr(f"Failed to load config: {e}")
				sys.exit()

		self._initalize_parameters()
		self._setup_publishers()
		self._setup_subscribers()
		self._initalize_detector()

	def _initalize_parameters(self):
		self.bridge = CvBridge()

		self.flag_publish_detected_image = self.config["publish_detected_image"]


		self.class_names = self.config["settings"]["class_names"]
		self.AT_ids = self.config["settings"]["AT_ids"]
		self.detection_confidence = self.config["settings"]["detection_confidence"]

	def _initalize_detector(self):
		self.detector = apriltags_detector.aprilTagDetector()

	
	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["image"], 
			Image, 
			self._new_image_callback
		)

	def _setup_publishers(self):
		if self.flag_publish_detected_image:
			self.detection_image_pub = rospy.Publisher(
				self.config["topics"]["output"]["detectionImage"], 
				Image, 
				queue_size=10
			)

		self.detection_boxes_pub = rospy.Publisher(
			self.config["topics"]["output"]["boxes"], 
			BoundingBoxes, 
			queue_size=10
		)

	def _new_image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		bbs, ids =  self.detector.find_april_tag(image)

		if len(bbs) != 0:
			confidence = [self.detection_confidence] * len(bbs)
			class_names = []
			for id in ids:
				index = self.AT_ids.index(id)
				class_names.append(self.class_names[index])

			if self.flag_publish_detected_image:
				detection_image = draw_detections(image.copy(), bbs, confidence, ids, class_names)

				self._publish_detected_image(detection_image)

			msg = self._prepare_boundingbox_msg(bbs, confidence, ids, class_names, image)
			self._publish_detected_boundingboxes(msg)


	def _publish_detected_image(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		self.detection_image_pub.publish(msg)

	def _prepare_boundingbox_msg(self, boxes, confidence, classes, class_names, image):
		boundingBoxes = BoundingBoxes()
		boundingBoxes.header.stamp = rospy.Time.now()

		for i in range(len(boxes)):
			boundingBox = BoundingBox()
			boundingBox.probability = confidence[i]
			boundingBox.xmin = int(boxes[i][0])
			boundingBox.ymin = int(boxes[i][1])
			boundingBox.xmax = int(boxes[i][2])
			boundingBox.ymax = int(boxes[i][3])
			boundingBox.id = int(classes[i])
			boundingBox.Class = class_names[i]

			boundingBoxes.bounding_boxes.append(boundingBox)

		boundingBoxes.frame = self.bridge.cv2_to_imgmsg(image, "bgr8")

		return boundingBoxes

	def _publish_detected_boundingboxes(self, msg):
		self.detection_boxes_pub.publish(msg)

	def _shutdown():
		rospy.loginfo("Shutting down detection_simulator node")

	def start(self):
		rospy.loginfo("Starting detection_simulator node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		detector = DetectorSimulator()
		detector.start()

if __name__ == "__main__":
		main()
