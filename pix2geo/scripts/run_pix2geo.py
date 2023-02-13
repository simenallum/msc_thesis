#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
from cv_bridge import CvBridge
import sensor_msgs.msg


from yolov8_ros.msg import BoundingBox, BoundingBoxes

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
		pass

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["bounding_boxes"], 
			BoundingBoxes, 
			self._new_bb_calback
		)

	def _setup_publishers(self):
		pass

	def _new_bb_calback(self, bounding_boxes):

		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)

	def _extract_bbs(self, bboxes):
		result = []
		for bbox in bboxes:
			left = bbox.xmin
			top = bbox.ymin
			width = bbox.xmax - bbox.xmin
			height = bbox.ymax - bbox.ymin
			result.append(([left, top, width, height], bbox.probability, bbox.Class))
		return result


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
