#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
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
			half=self.deepsort_params["half"],
			bgr=self.deepsort_params["bgr"],
			embedder_gpu=self.deepsort_params["embedder_gpu"],
			embedder_model_name=self.deepsort_params["embedder_model_name"],
			embedder_wts=self.deepsort_params["embedder_wts"],
			polygon=self.deepsort_params["polygon"],
			today=self.deepsort_params["today"],
			_lambda =self.deepsort_params["_lambda"],
		)

	def _new_bb_calback(self, bounding_boxes):
		start = time.time()

		bbs = self._extract_bbs(bounding_boxes.bounding_boxes)
		frame = self.bridge.imgmsg_to_cv2(bounding_boxes.frame, "bgr8")

		tracks = self.tracker.update_tracks(bbs, frame=frame)

		track_list = self._extract_bbs_and_trackid(tracks)


		image = draw_detections(frame, track_list)
		
		

		if self.flag_publish_images_with_tracks:
			self._publish_image_with_tracks(image)

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
