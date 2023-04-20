#! /usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os

class imageSaver():
		def __init__(self):
				self.i = 0

				self.bridge = CvBridge()

				self.last_image = None 

				self.script_dir = os.path.dirname(os.path.realpath(__file__))

				self.subfolder_name = rospy.get_param("/subfolder_name")

				self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
				
				rospy.Subscriber("/image_topic", Image, self.image_callback)

				isExist = os.path.exists(f"{self.script_dir}/../{self.subfolder_name}")
				if not isExist:

					# Create a new directory because it does not exist
					os.makedirs(f"{self.script_dir}/../{self.subfolder_name}")
					print("The new directory is created!")
				
		def image_callback(self, msg):
			self.last_image = msg

		def timer_callback(self, timer):
			try:
				
				# Check if no images has been received
				if self.last_image == None:
					return

				# Convert your ROS Image message to OpenCV2
				cv2_img = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")

        # Save your OpenCV2 image as a jpeg 
				cv2.imwrite(f"{self.script_dir}/../{self.subfolder_name}/outside_ses3_shadow_{self.i}.jpeg", cv2_img)
				self.i += 1
				print("IMAGE SAVED ", self.i)
				print("")
        
			except CvBridgeError:
				print("error saving image")
				pass
				


def main():
		rospy.init_node('image_listener')
		imgs = imageSaver()

		while not rospy.is_shutdown():
			rospy.spin()

if __name__ == '__main__':
		main()