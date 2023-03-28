#!/usr/bin/env python3

import rospy
import numpy as np

from scipy.spatial.transform import Rotation



from olympe_bridge.msg import CameraCommand

class cameraController():
	'''
	This class initialize the camera mounted on the gimbal to be oriented
	straight down in the world frame.
	'''

	def __init__(self, config_file=None):

		rospy.init_node("camera_control", anonymous=False)

		self.camera_command_publisher = rospy.Publisher(
			"/anafi/cmd_camera", CameraCommand, queue_size=10
		)

		rospy.sleep(2.0)

		self._publish_commmand(-90)
		rospy.sleep(2.0) # Giv the gimbal some time to rotate
		rospy.loginfo("Camera is inited to -90 deg in pitch")

	
	def _publish_commmand(self, pitch, header=None):
		msg = CameraCommand()
		msg.header.stamp = rospy.Time.now()

		msg.roll = 0
		msg.pitch = pitch
		msg.zoom = 0
		 
		self.camera_command_publisher.publish(msg)


	def start(self):

		rospy.loginfo("Starting camera_controller")

		while not rospy.is_shutdown():
			rospy.spin()
			


def main():
	estimator = cameraController()
	estimator.start()

if __name__ == "__main__":
	main()