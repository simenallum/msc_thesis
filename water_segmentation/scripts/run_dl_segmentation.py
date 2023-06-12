#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import time
from cv_bridge import CvBridge
import torch
import torch.nn.functional as F
import numpy as np
from PIL import Image
from torchvision import transforms
import cv2

from map_segmentation_utils import utils

from water_segmentation.srv import sendMask, sendMaskResponse
import sensor_msgs.msg
from Pytorch_segmentation.unet import UNet
from Pytorch_segmentation.segnet import SegNet_model
from Pytorch_segmentation.predict import predict_img, mask_to_image


class DL_segmentation:

	def __init__(self, config_file=None):

		rospy.init_node("DL_segmenation", anonymous=False)

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
		self._initialize_model()
		self._initalize_services()


	def _initalize_parameters(self):
		self.bridge = CvBridge()
		
		self.model_path = self.config['model_settings']['model_path']
		self.classes = self.config['model_settings']['classes']
		self.bilinear = self.config['model_settings']['bilinear']
		self.model_type = self.config['model_settings']['model_type']

		self._last_image = [None]
		self._debug = self.config['debug']

		self._segmentation_times = []

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["image"], 
			sensor_msgs.msg.Image, 
			self._new_image_cb
		)

	def _setup_publishers(self):

		self.mask_pub = rospy.Publisher(
			self.config["topics"]["output"]["water_mask"], 
			sensor_msgs.msg.Image, 
			queue_size=10
		)

	def _initialize_model(self):
		if self.model_type == "segnet":
			self.model = SegNet_model.SegNet(n_channels=3, n_classes=self.classes)
		elif self.model_type == "unet":
			self.model = UNet(n_channels=3, n_classes=self.classes, bilinear=self.bilinear)
		else:
			rospy.logerr(f"[dl_segmentation]: Uknown model type. Exiting!")
			rospy.signal_shutdown("Uknown model type!")

		self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
		self.model.to(device=self.device)
		self.state_dict = torch.load(self.model_path, map_location=self.device)
		self.mask_values = self.state_dict.pop('mask_values', [0, 1])
		self.model.load_state_dict(self.state_dict)

		rospy.loginfo("Model loaded!")

	def _initalize_services(self):
		self._srv_make_mask = rospy.Service(
			self.config["services"]["make_mask"],
			sendMask, 
			self._handle_create_mask
		)

	def _new_image_cb(self, image_msg):
		image_raw = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

		self._last_image = Image.fromarray(np.uint8(image_raw)).convert('RGB')
	
	def _handle_create_mask(self, req):
		# Record start time
		start_time = time.time()

		# Call predict_img() function
		mask = predict_img(
			net=self.model,
			full_img=self._last_image,
			scale_factor=0.5, # Scale parameter not used. Inference is performed at fixed 256x256 res
			out_threshold=0.5,
			device=self.device
		)

		# Record end time
		seg_time = time.time() - start_time
		self._segmentation_times.append(seg_time)
		rospy.logdebug("Time taken: {:.2f} seconds".format(seg_time))

		mask_image = mask_to_image(mask, self.mask_values)

		if self._debug:
			# Create and save debug image
			if not np.any(self._last_image) == None:
				masked_image = utils.apply_mask_overlay(np.asarray(self._last_image), mask_image)

				cv2.imwrite('/home/msccomputer/catkin_ws/src/msc_thesis/water_segmentation/data/debug/dl_overlays/{}.jpg'.format(time.strftime('%Y%m%d-%H%M%S')), masked_image)


		self._publish_mask_image(mask_image)

		# Make responce for service call
		res = sendMaskResponse()
		res.message = "Mask created!"
		res.image_data = self.bridge.cv2_to_imgmsg(mask_image, "mono8")
		return res


	def _publish_mask_image(self, mask):
		mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
		self.mask_pub.publish(mask_msg)

	def _shutdown(self):
		rospy.loginfo("Shutting down Deep Learning based Segmentation node")

		np_segtimes = np.array(self._segmentation_times)
		print("Mean segmenation times: ", np_segtimes.mean())
		print("Max segmenation time: ", np_segtimes.max())
		print("Median segmentation time: ", np.median(np_segtimes))

	def start(self):
		rospy.loginfo("Starting Deep Learning based Segmentation node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		Segmentation_worker = DL_segmentation()
		Segmentation_worker.start()

if __name__ == "__main__":
		main()
