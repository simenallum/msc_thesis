import numpy as np
import math
from scipy.spatial.transform import Rotation


def calculate_vfov(hfov, aspect_ratio):
	"""
	Calculates the vertical field of view (in degrees) of a camera given its horizontal field of view and aspect ratio.

	Args:
	- hfov (float): camera's horizontal field of view in degrees
	- aspect_ratio (float): camera's aspect ratio (width / height)

	Returns:
	- float: camera's vertical field of view in degrees
	"""

	# Convert hfov to radians
	hfov_rad = math.radians(hfov)

	# Calculate the vertical field of view in degrees using the diagonal field of view and aspect ratio
	vfov = math.degrees(2 * math.atan(math.tan(hfov_rad / 2) / aspect_ratio))

	return vfov

def get_bounding_box_center(bounding_box):
	left, top, width, height = bounding_box
	center_x = left + width / 2
	center_y = top + height / 2
	return [center_x, center_y]

def _pixel_to_camera_coordinates(center_px, drone_pos, camera_focal_length, image_center):
  # Image center
  x_c = image_center[0]
  y_c = image_center[1]

  # Distance from image center to object center
  d_x = x_c - center_px[0]
  d_y = y_c - center_px[1]

  # Find altitude from similar triangles
  z_camera = drone_pos[0]

  # Find x and y coordiantes using similar triangles as well. The signs are
  # used so that the x-coordinate is positive to the right and the y-coordinate
  # is positive upwards
  x_camera = -(z_camera * d_x / camera_focal_length)
  y_camera = -(z_camera * d_y / camera_focal_length)

  return x_camera, y_camera, z_camera


def calculate_detection_location(camera_fov, detection_pixels, drone_position, img_width, img_height):
	"""
	Calculate the location of a detected object in 3D space relative to a camera.

	Args:
	- camera_fov (tuple): (hfov, vfov) camera's field of view in degrees
	- detection_pixels (tuple): (x, y) location of detected object in pixels
	- drone_position (tuple): (longitude, latitude, altitude) of the drone's current position
	- img_width (int): width of the image in pixels
	- img_height (int): height of the image in pixels

	Returns:
	- tuple: (x, y, z) location of the detected object in meters relative to the drone in the camera coordinate system
	"""

	# The camera's field of view to horizontal and vertical field of view in degrees
	hfov, vfov = camera_fov

	# Calculate the angular displacement of the detected object from the center of the image
	x_displacement_degrees = detection_pixels[0] - (img_width / 2)
	y_displacement_degrees = detection_pixels[1] - (img_height / 2)

	# Calculate the angular resolution of the camera in degrees per pixel
	y_resolution_degrees_per_pixel = (vfov / img_height)
	x_resolution_degrees_per_pixel = (hfov / img_width)

	# Calculate the angular displacement of the detected object in degrees
	x_displacement_degrees_from_center = x_resolution_degrees_per_pixel * x_displacement_degrees
	y_displacement_degrees_from_center = y_resolution_degrees_per_pixel * y_displacement_degrees

	# Transform the coordinates to the camera coordinate system
	x_camera = (drone_position[0] * math.tan(math.radians(x_displacement_degrees_from_center)))
	y_camera = (drone_position[0] * math.tan(math.radians(y_displacement_degrees_from_center)))
	z_camera = drone_position[0]

	return (x_camera, y_camera, z_camera)

def transform_point_cam_to_world(point, translation, yaw_deg):
	'''
		Legacy function. 
		Manual transformation of points from cam to world.
	'''
	fixed_rotation = Rotation.from_euler('z', 90, degrees=True)
	yaw_rotation = Rotation.from_euler('z', yaw_deg, degrees=True)

	fixed_rotated_point = fixed_rotation.apply(point)
	translated_point = fixed_rotated_point + [0.07, 0.0, 0.0]
	rotated_point = yaw_rotation.apply(translated_point)

	point_new = rotated_point + [translation[0], translation[1], translation[2]]
	return point_new


def main():
		detection_pixels = (1280, 720)
		drone_position = (9.269724, 47.671949,  10) # Long, lat, alt
		drone_orientation = (45.4, 138.2)
		drone_pos_ned = (1, 1, 10)
		drone_velocity = (-0.39998927134555207, 0.39998927134555207, 0.299991953509164)
		mavic_hfov = 64.94
		mavic_vfov = 51.03
		img_width = 1280
		img_height = 720
		compass_heading = 0 #138.2

		anafi_hfov = 69
		anafi_vfoc = calculate_vfov(anafi_hfov, img_width / img_height)
		camera_fov = (mavic_hfov, mavic_vfov)


		det_cam = calculate_detection_location(camera_fov, detection_pixels, drone_position, img_width, img_height)
		print("CAM: ", det_cam)

		det_world = transform_point_cam_to_world(det_cam, drone_pos_ned, compass_heading)
		
		print("WOLRD: ", det_world)


if __name__ == "__main__":
		main()
