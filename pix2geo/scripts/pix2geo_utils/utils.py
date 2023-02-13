import numpy as np

def image_to_gnss(detection_pixels, drone_position, drone_orientation, drone_velocity, camera_fov, camera_aspect_ratio):
	# Calculate the field of view of the camera in the x and y direction
	fov_x = camera_fov
	fov_y = camera_fov / camera_aspect_ratio
	
	# Convert the detections from pixels to angles using the FOV values
	x_angle = detection_pixels[0] * fov_x / 2
	y_angle = detection_pixels[1] * fov_y / 2
	
	# Calculate the position of the object in the world coordinate system
	r = drone_position[2] / np.sin(y_angle)
	x_world = r * np.sin(x_angle) * np.cos(y_angle)
	y_world = r * np.cos(x_angle) * np.cos(y_angle)
	z_world = r * np.sin(y_angle)
	
	# Calculate the orientation of the camera in the world coordinate system
	c_pitch = np.cos(drone_orientation[0])
	s_pitch = np.sin(drone_orientation[0])
	c_heading = np.cos(drone_orientation[1])
	s_heading = np.sin(drone_orientation[1])
	
	R_pitch = np.array([[c_pitch, 0, -s_pitch], [0, 1, 0], [s_pitch, 0, c_pitch]])
	R_heading = np.array([[c_heading, s_heading, 0], [-s_heading, c_heading, 0], [0, 0, 1]])
	
	R = np.matmul(R_heading, R_pitch)
	
	# Project the position of the object from the world coordinate system to the camera coordinate system
	object_position_camera = np.matmul(R, np.array([x_world, y_world, z_world]))
	
	# Calculate the GNSS coordinate of the object
	gnss_coordinate = drone_position + object_position_camera
	
	return gnss_coordinate




def main():
		detection_pixels = (450, 340)
		drone_position = (9.269724, 47.671949,  8.599580615665955) # Long, lat, alt
		drone_orientation = (45.4, 138.2)
		drone_velocity = (-0.39998927134555207, 0.39998927134555207, 0.299991953509164)
		camera_fov = 77
		camera_aspect_ratio = 1280 / 720

		ret = image_to_gnss(detection_pixels, drone_position, drone_orientation, drone_velocity, camera_fov, camera_aspect_ratio)

		print(ret)

if __name__ == "__main__":
		main()
