import numpy as np
import math

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