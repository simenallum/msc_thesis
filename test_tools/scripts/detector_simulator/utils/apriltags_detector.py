#!/usr/bin/env python3

from pickletools import uint8
import cv2 as cv
import numpy as np
import apriltag
import matplotlib.pyplot as plt

class aprilTagDetector():

		def __init__(self):


				self.options = apriltag.DetectorOptions(
												families="tag36h11",
												border=1,
												nthreads=4,
												quad_decimate=1.0,
												quad_blur=0.0,
												refine_edges=True,
												refine_decode=False,
												refine_pose=True,
												debug=False,
												quad_contours=True)
				self.detector = apriltag.Detector(self.options)


		def preprocess_image(self, img):
			 
				output = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

				return output


		def find_april_tag(self, img):
				gray = self.preprocess_image(img)
				
				detections = self.detector.detect(gray, return_image=False)

				n = len(detections)
				
				bbs = []
				ids = []
				for i in range(n):
						tag_id = detections[i].tag_id
						xmin, ymin, xmax, ymax = self.corners_to_bbox(detections[i].corners)

						bbs.append(np.array([xmin, ymin, xmax, ymax]))
						ids.append(tag_id)
						
													
				return bbs, ids

		def corners_to_bbox(self, corners):
			x_min, y_min = np.min(corners, axis=0)
			x_max, y_max = np.max(corners, axis=0)
			return (x_min, y_min, x_max, y_max)

		

		def match_features(self, detections, metric_features):
				n = len(detections)
				img_features = np.array([], dtype=np.float32).reshape(2,0)
				metric_features_ret = np.array([], dtype=np.float32).reshape(4,0)
				
				for i in range(n):
						tag_id = detections[i].tag_id
						corners = np.transpose(detections[i].corners)

						img_features = np.hstack((img_features, corners))
						metric_features_ret = np.hstack((metric_features_ret, metric_features[:,tag_id*4:(tag_id*4)+4]))
								
				return img_features, metric_features_ret

def main():
		return

if __name__ == "__main__":
		main()