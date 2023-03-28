#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import glob
import time
import matplotlib.pyplot as plt


def get_valid_images(images, grid_size, visualize=False):
		"""
		Returns a list of the valid image filenames along with the object points
		in the 3D world space and in the image plane.
		"""

		# termination criteria
		criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		hehe = np.mgrid[0:grid_size[0], 0:grid_size[1]].T


		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		objp = np.zeros((num_corners, 3), np.float32)
		# objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)
		col = hehe[:,:,0].flatten()
		row = hehe[:,:,1].flatten()
		objp[:,0] = row
		objp[:,1] = col
		objp = objp * 0.034

		# Arrays to store object points and image points from all the images.
		objpoints = []  # 3d point in real world space
		imgpoints = []  # 2d points in image plane.
		valid_images = []  # list containing the filenames of the images where a chessboard was found

		if visualize:
				plt.figure()

		for fname in images:
				print(f"Finding chessboard in {fname}")
				img = cv.imread(fname)
				img = np.flip(img, 0)
				gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
				

				# Find the chess board corners
				ret, corners = cv.findChessboardCorners(gray, grid_size, None, cv.CALIB_CB_FAST_CHECK)

				# If found, add object points, image points (after refining them)
				if ret == True:
						valid_images.append(fname.split("/")[-1])
						objpoints.append(objp)
						corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
						imgpoints.append(corners)

						if visualize:
								# Draw and display the corners
								cv.drawChessboardCorners(img, grid_size, corners2, ret)
								plt.clf()
								plt.imshow(img[:, :, ::-1])
								plt.show(block=False)
								plt.pause(0.5)

		if visualize:
				plt.close()

		print(30*"=", "Finding chessboards", 29*"=")
		print(f"Found chessboards in {len(valid_images)}/{len(images)} images\n")

		return valid_images, objpoints, imgpoints


def calibrate(object_points, image_points, alpha, image_shape, output_dir, save):
		"""
		Returns
		- The standard camera matrix
		- The camera matrix when modifing the edges to account for distortion
		- The region of interest for the modified camera matrix
		- The distortion
		- List of rotational vectors
		- List of translational vectors
		- List of standard deviations of the intrinsic parameters
		"""

		ret, mtx, dist, rvecs, tvecs, std_deviation_intrinsic, _, _ = \
				cv.calibrateCameraExtended(object_points, image_points,
																	 (image_shape[1:3]), None, None)

		w = image_shape[1]
		h = image_shape[2]
		newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha, (w, h))

		if save:
				print("Saving output")
				np.savetxt(f"{output_dir}/camera_matrix.txt", mtx)
				np.savetxt(f"{output_dir}/camera_matrix_corrected.txt", newcameramtx)
				np.savetxt(f"{output_dir}/distortion.txt", dist)
				np.savetxt(f"{output_dir}/roi.txt", roi)
				np.savetxt(f"{output_dir}/std_deviation_intrinsic.txt", std_deviation_intrinsic)

				with open(f"{output_dir}/rotation_vecs.txt", "wb") as f:
						for rvec in rvecs:
								np.savetxt(f, rvec)
								f.write(b"\n")

				with open(f"{output_dir}/translation_vecs.txt", "wb") as f:
						for tvec in tvecs:
								np.savetxt(f, tvec)
								f.write(b"\n")

		return mtx, newcameramtx, roi, dist, rvecs, tvecs, std_deviation_intrinsic


def calculate_errors(object_points, image_points, rvecs, tvecs, camera_matrix,
										 distortion, std_deviation_intrinsic, num_corners):
		# Calculate mean reprojection errors for all calibration images
		all_errors = []

		for i in range(len(object_points)):
				image_points2, _ = cv.projectPoints(
						object_points[i], rvecs[i], tvecs[i], camera_matrix, distortion)
				errors = image_points[i] - image_points2
				all_errors.append(errors)

		image_errors = [cv.norm(errors, cv.NORM_L2)/num_corners for errors in all_errors]
		mean_error = np.mean(image_errors)
		print(27*"=", f"Mean reprojection errors", 27*"=")
		print(f"Total error all images (pixels): {mean_error}\n")
		for i in range(len(image_errors)):
				print(f"Reprojection error for image {valid_images[i]}: {image_errors[i]}")

		# Plot reprojection errors per image
		# x_labels = [img[:-4] for img in valid_images]  # remove .jpg
		x_labels = np.arange(0, len(image_errors), 1)
		plt.figure()
		plt.bar(x_labels, image_errors)
		plt.xticks(np.arange(len(image_errors)), x_labels, rotation="vertical")
		plt.title(f"Mean reprojection error (all images): {mean_error:.6f}")

		# Plot all errors
		plt.figure()
		for errors in all_errors:
				x_err = errors[:, 0, 0]
				y_err = errors[:, 0, 1]
				scatter = plt.scatter(x_err, y_err)

		# plt.legend(valid_images)
		plt.title("Reprojection error for all reprojections")

		# Distortion coeffs
		fx = camera_matrix[0, 0]
		fy = camera_matrix[1, 1]
		cx = camera_matrix[0, 2]
		cy = camera_matrix[1, 2]
		k1 = distortion[0, 0]
		k2 = distortion[0, 1]
		p1 = distortion[0, 2]
		p2 = distortion[0, 3]
		k3 = distortion[0, 4]

		# Print intrinsic standard deviations
		std_fx = std_deviation_intrinsic[0, 0]
		std_fy = std_deviation_intrinsic[1, 0]
		std_cx = std_deviation_intrinsic[2, 0]
		std_cy = std_deviation_intrinsic[3, 0]
		std_k1 = std_deviation_intrinsic[4, 0]
		std_k2 = std_deviation_intrinsic[5, 0]
		std_p1 = std_deviation_intrinsic[6, 0]
		std_p2 = std_deviation_intrinsic[7, 0]
		std_k3 = std_deviation_intrinsic[8, 0]

		print("\n" + 34*"=", "Intrinsics", 34*"=")
		print(
				f"Focal length (pixels): \n\tfx = {fx:.2f} +/- {std_fx:.2f} \n\tfy = {fy:.2f} +/- {std_fy:.2f}")
		print(
				f"Optical centers/principal point (pixels): \n\tcx = {cx:.2f} +/- {std_cx:.2f} \n\tcy = {cy:.2f} +/- {std_cy:.2f}")
		print(
				f"Radial distortion: \n\tk1 = {k1:.2f} +/- {std_k1:.2f} \n\tk2 = {k2:.2f} +/- {std_k2:.2f} \n\tk3 = {k3:.2f} +/- {std_k3:.2f}")
		print(
				f"Tangential distortion: \n\tp1 = {p1:.2f} +/- {std_p1:.2f} \n\tp2 = {p2:.2f} +/- {std_p2:.2f}")
		print(80*"=")


if __name__ == "__main__":
		import os
		import sys

		usage = "Usage: calibrate.py <save/nosave>"

		if len(sys.argv) != 2:
				print(usage)
				sys.exit()

		if sys.argv[1] == "save":
				save = True
		elif sys.argv[1] == "nosave":
				save = False
		else:
				print(usage)
				sys.exit()

		os.chdir(os.path.dirname(os.path.abspath(__file__)))

		grid_size = (7, 10)
		num_corners = grid_size[0] * grid_size[1]
		base_path = "/home/msccomputer/catkin_ws/src/anafi_perception/apriltags_pose_estimator/data/lab_calib_images"
		timestr = time.strftime("/%Y%m%d-%H%M%S")
		output_path = base_path + timestr
		os.mkdir(output_path)

		images = glob.glob(f"{base_path}/*.jpeg")
		images.sort(key=lambda x: x[-8:-4])  # sort images based on last numbers

		img_shape = cv.imread(images[0]).shape[::-1]
		print(img_shape)

		valid_images, object_points, image_points = \
				get_valid_images(images, grid_size, visualize=False)

		alpha = 1
		camera_matrix, new_camera_matrix, roi, distortion, rvecs, tvecs, \
				std_deviation_intrinsic = calibrate(object_points, image_points,
																						alpha, img_shape, output_path, save)

		calculate_errors(object_points, image_points, rvecs, tvecs, camera_matrix,
										 distortion, std_deviation_intrinsic, num_corners)

		plt.show()