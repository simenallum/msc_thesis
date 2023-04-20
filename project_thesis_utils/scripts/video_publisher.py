#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge, CvBridgeError

from yolov8_ros.msg import BoundingBox, BoundingBoxes

class VideoPublisherNode:
    def __init__(self):
        self.video_path = rospy.get_param("~video_path")
        self.topic_name = rospy.get_param("~topic_name")
        self.bb_topic_name = rospy.get_param("~bb_topic_name")
        self.track_gt_topic_name = rospy.get_param("~track_gt_topic_name")
        self.annotations_folder_path = rospy.get_param("~annotations_folder_path")
        self.video_name = rospy.get_param("~video_name")
        self.fps = rospy.get_param("~fps")
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.bb_pub = rospy.Publisher(self.bb_topic_name, BoundingBoxes, queue_size=10)
        self.gt_pub = rospy.Publisher(self.track_gt_topic_name, BoundingBoxes, queue_size=10)

        self.video_capture = cv2.VideoCapture(self.video_path)

        self.folder_path = self.annotations_folder_path 
        self.prefix = self.video_name

        self.filenames = [filename for filename in os.listdir(self.folder_path) if filename.startswith(self.prefix)]
        self.filenames.sort()

        self.class_labels = ["human", "floating-object"]


    def run(self):
        rate = rospy.Rate(self.fps)
        i = 0
        f_count = 0
        while not rospy.is_shutdown():
            ret, frame = self.video_capture.read()
            h = frame.shape[0]
            w = frame.shape[1]
            frame = cv2.resize(frame, (1280, 720)) # Convert resolution to 720p
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.image_pub.publish(msg)

                    if int(self.filenames[f_count].split("_")[-1].split(".")[0]) == i:

                        with open(os.path.join(self.folder_path, self.filenames[f_count]), "r") as f:
                            lines = f.readlines()
                        
                        if f_count % 1 == 0:
                            boundingBoxes = BoundingBoxes()
                            boundingBoxes.header = msg.header
                            bbs = []
                            for line in lines:
                                line = [float(num) for num in line.split()]
                                boundingBox = BoundingBox()
                                boundingBox.probability = 0.8

                                line[1] = (line[1] / w) * 1280
                                line[2] = (line[2] / h) * 720
                                line[3] = (line[3] / w) * 1280
                                line[4] = (line[4] / h) * 720

                                boundingBox.xmin = int(line[1])
                                boundingBox.ymin = int(line[2])
                                boundingBox.xmax = int(line[3])
                                boundingBox.ymax = int(line[4])
                                boundingBox.id = int(line[0])
                                boundingBox.Class = self.class_labels[int(line[0])]

                                bbs.append(boundingBox)

                            boundingBoxes.bounding_boxes = bbs

                        # Pub GT
                        boundingBoxesGT = BoundingBoxes()
                        boundingBoxesGT.header = msg.header
                        bbs = []
                        for line in lines:
                            line = [float(num) for num in line.split()]
                            boundingBox = BoundingBox()
                            boundingBox.probability = 0.8

                            line[1] = (line[1] / w) * 1280
                            line[2] = (line[2] / h) * 720
                            line[3] = (line[3] / w) * 1280
                            line[4] = (line[4] / h) * 720

                            try:
                                track_id = -line[5]
                            except:
                                track_id = 999

                            boundingBox.xmin = int(line[1])
                            boundingBox.ymin = int(line[2])
                            boundingBox.xmax = int(line[3])
                            boundingBox.ymax = int(line[4])
                            boundingBox.id = int(track_id)
                            boundingBox.Class = self.class_labels[int(line[0])]

                            bbs.append(boundingBox)

                        boundingBoxesGT.bounding_boxes = bbs
                        self.gt_pub.publish(boundingBoxesGT)

                        if not f_count >= len(self.filenames)-1:
                            f_count += 1

                    else:
                        boundingBoxes = BoundingBoxes()
                        boundingBoxes.header = msg.header
                    
                    boundingBoxes.frame = msg
                    self.bb_pub.publish(boundingBoxes)

                except CvBridgeError as e:
                    rospy.logerr("Error converting frame to image message: %s", e)
            else:
                rospy.logwarn("No more frames in video, stopping video publishing")
                break

            i += 1
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('video_publisher')
    node = VideoPublisherNode()
    node.run()
