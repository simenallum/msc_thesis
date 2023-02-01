#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
from cv_bridge import CvBridge


from deepsort_realtime.deepsort_tracker import DeepSort
from yolov8_ros.msg import BoundingBox, BoundingBoxes
from deepsort_tracker_utils.bounding_box_utils import convert_bboxes, draw_detections
import sensor_msgs.msg

class DeepSortTracker:

	def __init__(self, config_file=None):

		rospy.init_node("DeepSortTracker", anonymous=False)

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

		self.tracker = DeepSort(
			max_age=30,
			n_init=30,
			nms_max_overlap=0.7,
			max_iou_distance=0.7,
			max_cosine_distance=0.2,
			nn_budget=None,
			override_track_class=None,
			embedder="clip_RN50x16",
			half=True,
			bgr=True,
			embedder_gpu=True,
			embedder_model_name=None, 
			embedder_wts=None,
			polygon=False,
			today=None,
		)

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

	def _new_bb_calback(self, bounding_boxes):
		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)
		frame = self.bridge.imgmsg_to_cv2(bounding_boxes.frame, "bgr8")

		tracks = self.tracker.update_tracks(bbs, frame=frame)

		track_list = self._extract_bbs_and_trackid(tracks)


		image = draw_detections(frame, track_list)
		
		

		if self.flag_publish_images_with_tracks:
			self._publish_image_with_tracks(image)

	def _publish_image_with_tracks(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		self.image_with_tracks_pub.publish(msg)

	def _extract_bbs(self, bounding_boxes):
		return convert_bboxes(bounding_boxes)

	def _extract_bbs_and_trackid(self, tracks):
		result = []

		for track in tracks:
			bb = track.to_tlbr(orig=False)
			conf = track.get_det_conf() or 0
			id = track.track_id

			result.append([bb, conf, id])

		return result

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
