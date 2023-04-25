import numpy as np
import math
import pymap3d
import csv

def is_point_within_threshold_dict(point, threshold, dictionary: dict):
    for value in dictionary.values():
        distance = calculate_euclidian_distance(point, value)
        if distance < threshold:
            return True
    return False

def is_point_within_threshold_list(point, threshold, points: list):
    for value in points:
        distance = calculate_euclidian_distance(point, value)
        if distance < threshold:
            return True
    return False

def key_exists(key, dictionary):
    return key in dictionary

def calculate_euclidian_distance(point1: np.array, point2: np.array) -> float:
    dist = np.linalg.norm(point1 - point2)

    return dist

def calculate_euclidian_distance_of_vector(vec1: np.array) -> float:
    dist = np.linalg.norm(vec1)

    return dist

def get_closest_distance(point, dictionary):
    closest_distance = float('inf')
    for value in dictionary.values():
        distance = calculate_euclidian_distance(point, value)
        if distance < closest_distance:
            closest_distance = distance
    return closest_distance

def calculate_critical_level(distance, critical_levels):
    if distance >= critical_levels[2]:
        return 2
    
    if distance >= critical_levels[1]:
        return 1
    
    return 0

def get_dict_values_as_list(dict):
    ret_list = []
    for value in dict.values():
        ret_list.append(value)

    return ret_list

def convert_ned_list_to_gnss_list(ned_frame_origin, ned_list):

    ell_wgs84 = pymap3d.Ellipsoid('wgs84')
    lat_origin, lon_origin, h_origin = ned_frame_origin

    gnss_list = []
    for points in ned_list:
        lat1, lon1, h1 = pymap3d.ned2geodetic(points[0], points[1], points[2], \
                                            lat_origin, lon_origin, h_origin, \
                                            ell=ell_wgs84, deg=True)  # wgs84 ellisoid
        
        gnss_list.append([lat1, lon1, h1])

    return gnss_list

def save_np_list_as_csv(outfile, np_array):
    np.savetxt(outfile, np_array, delimiter=',', newline="\n")

def calculate_ground_coverage(camera_fov: tuple, altitude: float):
    hfov = camera_fov[0]
    vfov = camera_fov[1]

    h_coverage = (altitude * np.tan(np.deg2rad(hfov / 2))) * 2
    v_coverage = (altitude * np.tan(np.deg2rad(vfov / 2))) * 2

    return (h_coverage, v_coverage)

def get_fov_from_hfov(image_width, image_height, hfov):
	aspect_ratio = image_width / image_height
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)