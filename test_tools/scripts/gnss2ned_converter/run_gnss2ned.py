#!/usr/bin/env python3

import rospy
import os
import yaml
import sys
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import pymap3d


class GNSS2NED:

	def __init__(self, config_file=None, deepsort_params=None):

		rospy.init_node("gnss2NED", anonymous=False)

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
		self.ned_origo_in_lla = None

	def _setup_subscribers(self):
		rospy.Subscriber(
			self.config["topics"]["input"]["GNSS"], 
			NavSatFix, 
			self._new_gnss_data_callback
		)

	def _setup_publishers(self):
		self.pub_ned_pos_from_gnss = rospy.Publisher(
			self.config["topics"]["output"]["NED"], 
			PointStamped, 
			queue_size=10
		)

	def _new_gnss_data_callback(self, msg):

		n, e, d = self._calculate_ned_position_wgs84(msg)

		msg_ned_pos_from_gnss = PointStamped()
		msg_ned_pos_from_gnss.header = msg.header 
		msg_ned_pos_from_gnss.point.x = n
		msg_ned_pos_from_gnss.point.y = e 
		msg_ned_pos_from_gnss.point.z = d 

		self.pub_ned_pos_from_gnss.publish(msg_ned_pos_from_gnss)

	def _calculate_ned_position_wgs84(self, gnss_msg : NavSatFix) -> np.ndarray:

		ell_wgs84 = pymap3d.Ellipsoid('wgs84')
		lat, lon, a = gnss_msg.latitude, gnss_msg.longitude, gnss_msg.altitude

		if self.ned_origo_in_lla is None:
			self.ned_origo_in_lla = (lat, lon, 0)

		lat_0 = self.ned_origo_in_lla[0]
		lon_0 = self.ned_origo_in_lla[1]
		a_0 = self.ned_origo_in_lla[2]

		return pymap3d.geodetic2ned(lat, lon, a, lat_0, lon_0, a_0, ell=ell_wgs84, deg=True)


	def _shutdown():
		rospy.loginfo("Shutting GNSS2NED node")

	def start(self):
		rospy.loginfo("Starting GNSS2NED node")

		rospy.on_shutdown(self._shutdown)

		while not rospy.is_shutdown():

			rospy.spin()


def main():
		converter = GNSS2NED()
		converter.start()

if __name__ == "__main__":
		main()
