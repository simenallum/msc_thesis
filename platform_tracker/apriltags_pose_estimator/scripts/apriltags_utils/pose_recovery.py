
from sys import flags
import cv2 as cv
import numpy as np
import tf2_ros as tf2
import rospy

from . import homography

from tf2_geometry_msgs import PoseStamped
from scipy.spatial.transform import Rotation


class PoseRecovery():

		def __init__(self, K, camera_offsets, offline=False):
			self.K = K
			self.camera_offsets = camera_offsets

			if not offline:
				self.tfBuffer = tf2.Buffer()
				self.listener = tf2.TransformListener(self.tfBuffer)

		def find_R_t_pnp(self, object_points, image_points):
			distortion_coeffs = np.zeros((5,1))

			# cv.solvePnP(...) requires object points to be of shape (n_points, 3)
			# and image points to be of shape (n_points, 2)
			object_points = homography.dehomogenize(object_points.copy()).T.reshape(-1, 1, 3)
			image_points = image_points.T.reshape(-1, 1, 2)

			_, R_vec, t_vec = cv.solvePnP(object_points, image_points, self.K, distortion_coeffs, flags=cv.SOLVEPNP_IPPE)
			
			Rt = Rotation.from_rotvec(R_vec.reshape(3,))
			R = Rt.as_matrix()
			
			t = t_vec.reshape(3,) # This is the platform coordinates origin given in camera coordinates

			return R, t

		def camera_to_drone_body_frame(self, R_camera, t_camera):
			rot = Rotation.from_matrix(R_camera)
			quat = rot.as_quat()

			pose = PoseStamped()
			pose.header.frame_id = "camera"
			pose.header.stamp = rospy.Time()

			pose.pose.position.x = t_camera[0]
			pose.pose.position.y = t_camera[1]
			pose.pose.position.z = t_camera[2]

			pose.pose.orientation.x = quat[0]
			pose.pose.orientation.y = quat[1]
			pose.pose.orientation.z = quat[2]
			pose.pose.orientation.w = quat[3]

			body_pose = self.tfBuffer.transform(pose, "body", rospy.Duration(0.1))

			t_body = np.array([body_pose.pose.position.x, body_pose.pose.position.y, body_pose.pose.position.z])

			R_body = (Rotation.from_quat([body_pose.pose.orientation.x, body_pose.pose.orientation.y, body_pose.pose.orientation.z, body_pose.pose.orientation.w])).as_matrix()

			return R_body, t_body


		def find_R_t_homography(self, features_metric, features_image):
			XY01 = features_metric.copy()
			uv01 = features_image.copy()


			H = self.find_homography(features_image, features_metric)

			T1, T2 = homography.decompose_H(H)
			T = homography.choose_decomposition(T1, T2, XY01)

			R = T[:3, :3]
			t = T[:3, 3]

			return R, t

		def get_pose_from_R_t(self, R, t):
				# The orientation angles are probably incorrect, but this has not been investigated
				# as only the position is used in the EKF.
				orientation = homography.rotation_matrix2euler_angles(R)*180/np.pi
				pos = t
				pose = np.hstack((pos, orientation))
				return pose

		def find_homography(self, features_image, features_metric):
			# uv is image pixel location (origin = upper left)
			uv = features_image.copy()
			uv1 = homography.homogenize(uv)

			# xy is image coordinates with origin = center
			xy_homgen = np.linalg.solve(self.K, uv1)
			xy = homography.dehomogenize(xy_homgen)

			# XY are the metric coordinates in the helipad frame
			XY = features_metric[0:2].copy()

			H = homography.estimate_H_opencv(xy.T, XY.T)

			return H

def main():
		return

if __name__ == "__main__":
		main()