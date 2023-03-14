import numpy as np
import math
from skimage.util import view_as_windows

def get_fov_from_hfov(image_width, image_height, hfov):
	aspect_ratio = image_width / image_height
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)

def dice_coefficient(mask1: np.array, mask2: np.array) -> float:
    """
    Computes the Dice coefficient between two binary masks.
    
    Parameters:
        mask1 (np.ndarray): The first binary mask as a numpy array with values of 0 or 255.
        mask2 (np.ndarray): The second binary mask as a numpy array with values of 0 or 255.
        
    Returns:
        float: The Dice coefficient between the two masks.
    """
    # Convert the mask values to binary (i.e., 0s and 1s)
    mask1 = mask1.astype(bool)
    mask2 = mask2.astype(bool)
    
    intersection = np.logical_and(mask1, mask2)
    dice_coeff = 2 * np.sum(intersection) / (np.sum(mask1) + np.sum(mask2))
    return dice_coeff


def convert_save_dist_to_px(focal_length: float, drone_altitude: float, safe_metric_distance: float) -> int:
    # Using similar triangles
    return int(safe_metric_distance * focal_length / drone_altitude)


def find_safe_areas(image: np.ndarray, window_size: int, stride: int) -> np.ndarray:
    # Define a sliding window of size window_size
    window = np.ones((window_size, window_size), dtype=np.uint8)

    binary_img = np.where(image > 0, 0, 1)  # Invert image and make binary
    center = np.array(binary_img.shape) / 2

    # Crop a subarray centered at the image center and of size window_size times window_size
    center_window = binary_img[
        int(center[0] - window_size / 2):int(center[0] + window_size / 2),
        int(center[1] - window_size / 2):int(center[1] + window_size / 2)
    ]

    # Check if the center window is safe
    if np.sum(np.multiply(center_window, window)) == window_size ** 2:
        return np.array([center])

    # Split the binary image into windows
    windows = view_as_windows(binary_img, window.shape, step=stride)

    safe_loc = []
    for i in range(len(windows)):
        for j in range(len(windows[i])):
            res = np.multiply(windows[i, j], window)
            if np.sum(res) == window_size ** 2:
                safe_loc.append([(i * stride) + (window_size // 2), (j * stride) + (window_size // 2)])

    safe_loc = np.array(safe_loc)

    # Calculate the distances of the safe windows to the center of the image
    if len(safe_loc != 0):
      distances = np.linalg.norm(safe_loc - center, axis=1)

      # Sort the safe windows by their distance to the center of the image
      sorted_indices = np.argsort(distances)
      sorted_positions = safe_loc[sorted_indices]

      return sorted_positions

    else:
      return None


if __name__ == '__main__':
    import cv2
    import matplotlib.pyplot as plt

    focal_length = 932.8763
    drone_alt = 40
    safe_metric_dist = 5
    stride = 10


    px_safe_dist = convert_save_dist_to_px(focal_length, drone_alt, safe_metric_dist)
    print(stride)

    # Example usage with a square of size 10 pixels
    img = cv2.imread("/home/msccomputer/Desktop/test10_0.png", 0)
    sorted = find_safe_areas(img, px_safe_dist, stride)
    plt.imshow(img)
    plt.plot(sorted[0][1], sorted[0][0], 'o', ms=3, c='r')
    plt.plot(img.shape[0] // 2, img.shape[0] // 2, 'o', ms=3, c='g')
    plt.legend(["Safe location", "Image center"])
    plt.show()