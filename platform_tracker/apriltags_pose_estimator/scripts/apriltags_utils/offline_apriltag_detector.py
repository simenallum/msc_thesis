import cv2
from apriltags_detector import aprilTagDetector

'''
This file initially used to display AT IDs and corner IDs. 
'''

def main():
		capture = cv2.VideoCapture(0)

		detector = aprilTagDetector()

		while (True):
				ret, frame = capture.read()

				overlay, detections = detector.find_april_tag(frame)

				detector.show_tag_corners_found(frame, detections)
				detector.show_centers_found(frame, detections)
		
				if cv2.waitKey(1) == 27:
						break
 
		capture.release()
 
		cv2.destroyAllWindows()

		return


if __name__ == "__main__":
		main()