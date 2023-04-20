#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
        self.image_folder = '/home/simenallum/development/aofdataset-2/test/images/'
        self.frequency = 0.25  # publish frequency in Hz

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            for image_path in os.listdir(self.image_folder):
                # check if file is an image
                if '.jpg' in image_path or '.png' in image_path:
                    image = cv2.imread(os.path.join(self.image_folder, image_path))
                    ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    self.image_pub.publish(ros_image)
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node('image_publisher')
    image_publisher = ImagePublisher()
    image_publisher.run()
