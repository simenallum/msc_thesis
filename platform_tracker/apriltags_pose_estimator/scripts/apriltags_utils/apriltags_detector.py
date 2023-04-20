#!/usr/bin/env python3

from pickletools import uint8
import cv2 as cv
import numpy as np
import apriltag
import matplotlib.pyplot as plt
from . import homography

class aprilTagDetector():

		def __init__(self, img_width, img_height):
				self.img_width = img_width
				self.img_height = img_height

				self.options = apriltag.DetectorOptions(families="tag36h11",
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

				try:
					detections = self.detector.detect(gray, return_image=False)
				except Exception as e:
					print(f"Exception in AprilTag detection: {e}")
					

				return detections

		def show_tag_corners_found(self, img, detections, color="red", window_name="Detected april tags corners", offline=False):
				image = np.copy(img)

				if color == "red":
						c = (0,0,255)
				elif color == "blue":
						c = (255,0,0)

				for i in range(len(detections)):
						for corner_id in range(len(detections[i].corners)):
								corner = (int(detections[i].corners[corner_id][0]), int(detections[i].corners[corner_id][1]))

								text_face = cv.FONT_HERSHEY_DUPLEX
								text_scale = 0.5
								text_thickness = 1
								text = str(corner_id)
								text_offset = 10

								text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
								text_origin = (
										int(corner[0] - text_size[0] / 2) + text_offset,
										int(corner[1] + text_size[1] / 2) - text_offset
								)

								cv.circle(image, corner, 4, c, cv.FILLED)
								cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

				if not offline: 
					cv.imshow(window_name, image)
					cv.waitKey(1)

				return image

		def show_centers_found(self, img, detections, color="red", window_name="Detected aprilTag centers", offline=False):
				image = np.copy(img)

				if color == "red":
						c = (0,0,255)
				elif color == "blue":
						c = (255,0,0)

				for i in range(len(detections)):
						center = (int(detections[i].center[0]), int(detections[i].center[1]))

						text_face = cv.FONT_HERSHEY_DUPLEX
						text_scale = 0.5
						text_thickness = 1
						text = str(detections[i].tag_id)
						text_offset = 10

						text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
						text_origin = (
								int(center[0] - text_size[0] / 2) + text_offset,
								int(center[1] + text_size[1] / 2) - text_offset
						)

						cv.circle(image, center, 4, c, cv.FILLED)
						cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

				if not offline: 
					cv.imshow(window_name, image)
					cv.waitKey(1)

				return image

		
		def show_image(self, img, window_name="WINDOW NOT NAMED"):
				image = np.copy(img)

				cv.imshow(window_name, image)
				cv.waitKey(1)

		def plot_axis_of_apriltags(self, overlay, feature_metric, R_cam, t_cam):
				opoints = homography.dehomogenize(feature_metric.copy()).T.reshape(-1, 1, 3)

				dcoeffs = np.zeros(5)

				ipoints, _ = cv.projectPoints(opoints, R_cam, t_cam, self.K, dcoeffs)

				ipoints = np.round(ipoints).astype(int)

				ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

				cv.line(overlay, ipoints[0], ipoints[1], (0,0,255), 2)
				cv.line(overlay, ipoints[1], ipoints[2], (0,255,0), 2)
				cv.line(overlay, ipoints[1], ipoints[3], (255,0,0), 2)
				font = cv.FONT_HERSHEY_SIMPLEX
				cv.putText(overlay, 'X', ipoints[0], font, 0.5, (0,0,255), 2, cv.LINE_AA)
				cv.putText(overlay, 'Y', ipoints[2], font, 0.5, (0,255,0), 2, cv.LINE_AA)
				cv.putText(overlay, 'Z', ipoints[3], font, 0.5, (255,0,0), 2, cv.LINE_AA)

				cv.imshow("Coordinate system of discovered apriltags", overlay)
				cv.waitKey(1)

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