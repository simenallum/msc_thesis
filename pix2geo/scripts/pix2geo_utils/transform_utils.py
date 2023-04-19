
from sys import flags
import cv2 as cv
import numpy as np
import tf2_ros as tf2
import rospy

from tf2_geometry_msgs import PoseStamped
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped

class Transformer():

		def __init__(self):

			self.tfBuffer = tf2.Buffer()
			self.listener = tf2.TransformListener(self.tfBuffer)

		def get_height_from_timestamp(self, timestamp):
			point = PointStamped()
			point.header.frame_id = "ground_alt"
			point.header.stamp = timestamp

			point.point.x = 0
			point.point.y = 0
			point.point.z = 0

			try:
				altitude_point = self.tfBuffer.transform(point, "drone_alt", rospy.Duration(0.05)) # TF should be available at 30Hz. 
				return altitude_point.point.z
			except Exception as e:
				return None
			

		def camera_to_world_frame(self, point_list, timestamp):
			point = PointStamped()
			point.header.frame_id = "camera"
			point.header.stamp = timestamp

			point.point.x = point_list[0]
			point.point.y = point_list[1]
			point.point.z = point_list[2]



			try:
				world_point = self.tfBuffer.transform(point, "world", rospy.Duration(0.05)) # TF should be available at 30Hz. 
				return [world_point.point.x, world_point.point.y, world_point.point.z]
			except:
				return [None]



def main():
		return

if __name__ == "__main__":
		main()