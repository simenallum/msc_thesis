import osmnx as ox
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
import geopandas as gpd
import matplotlib.pyplot as plt
import cv2
import rasterio
from rasterio.features import geometry_mask
import numpy as np
import math

def read_gdp_from_file(file_path: str) -> gpd.GeoDataFrame:
    return gpd.read_file(file_path)

def calculate_gnss_bbox(origin: tuple, radius: float) -> gpd.GeoDataFrame:
    
    # Calculate the bounding box using the center point and radius
    bounds = ox.utils_geo.bbox_from_point(origin, dist=radius)
    north, south, east, west = bounds

    # Create a polygon geometry from the bounding box coordinates
    bbox_polygon = Polygon([(west, north), (east, north), (east, south), (west, south)])

    large_scale_bbx_gpd = gpd.GeoDataFrame(geometry=[bbox_polygon], crs="EPSG:4326")

    # Return both the bbox and the bbox bounds
    return large_scale_bbx_gpd

def calculate_map_intersections(bbox_gpd: gpd.GeoDataFrame, multipolygon_gpd: gpd.GeoDataFrame) -> gpd.GeoDataFrame:

    # extract the bbox polygon from frame. ( Assuming there is only 1 bbox )
    bbox_polygon = bbox_gpd.geometry[0]

    # create a spatial index for frame with mutiple polygons
    sindex = multipolygon_gpd.sindex

    # find the polygons in multipoly frame that intersect with bbox
    possible_matches_index = list(sindex.intersection(bbox_polygon.bounds))
    possible_matches = multipolygon_gpd.iloc[possible_matches_index].loc[multipolygon_gpd.intersects(bbox_polygon)]

    # compute the intersection between bbox and all polygons in possible_matches
    intersections = possible_matches.intersection(bbox_polygon)

    # compute the union of the resulting polygons
    new_polygon = unary_union(intersections)

    # create a new GeoDataFrame with the new polygon
    large_scale_map_gpd = gpd.GeoDataFrame({'geometry': [new_polygon]}, crs=bbox_gpd.crs)

    return large_scale_map_gpd


def plot_gpf_frame(frame: gpd.GeoDataFrame) -> None:
    frame.plot()
    plt.grid()
    plt.show()

def calculate_mask_from_gpd(bbox_frame: gpd.GeoDataFrame, water_polygon_frame: gpd.GeoDataFrame, mask_resolution: tuple = (2000, 2000)) -> np.array:

    # Extract the polygon from the water polygon frame
    water_polygon = water_polygon_frame.geometry[0]

    # Extract the bbox polygon from the GeoPandas DataFrame
    bbox_polygon = bbox_frame.geometry[0]

    # Define the resolution of the mask
    mask_width, mask_height = mask_resolution

    # Define the bounding box of the mask (in the same CRS as the polygon)
    bounds = bbox_polygon.bounds
    xmin, ymin, xmax, ymax = bounds
    xres = (xmax - xmin) / mask_width
    yres = (ymax - ymin) / mask_height

    # Create an empty numpy array to store the mask
    mask = np.zeros((mask_height, mask_width), dtype=np.uint8)

    # Convert the polygon to a binary mask
    mask_img = geometry_mask(geometries=[water_polygon],
                out_shape=mask.shape,
                transform=rasterio.Affine(xres, 0.0, xmin, 0.0, -yres, ymax),
                invert=True,
                all_touched=True,
                )

    return mask_img

def rotate_image_mask(mask: np.array, yaw_deg: float) -> np.array:
    # Load the image as a NumPy UInt8 array
    img = mask.astype('uint8')

    # Define the rotation angle in degrees
    angle = yaw_deg

    # Get the image dimensions
    width, height = img.shape[:2]

    # Calculate the rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((width/2, height/2), angle, 1)

    # Apply the rotation to the image
    cv2_img =  cv2.warpAffine(img, rotation_matrix, (width, height))

    return cv2_img

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

def extract_metric_map(mask: np.array, map_resolution: tuple, map_radius: float, ground_coverage: tuple) -> np.array:
    # Get the image dimensions
    width, height = mask.shape[:2]

    pixels_per_meter_x = (map_radius*2)/map_resolution[0]
    pixels_per_meter_y = (map_radius*2)/map_resolution[1]

    # Define the number of pixels to take on each side of the center
    n_x = int(ground_coverage[0] / pixels_per_meter_x)
    n_y = int(ground_coverage[1] / pixels_per_meter_y)

    # Divide by two to get numbr of px in each direction from center
    n_x = n_x // 2
    n_y = n_y // 2

    # Calculate the x and y coordinates of the center of the image
    center_x = width // 2
    center_y = height // 2

    # Calculate the coordinates of the top-left corner of the new image
    start_x = center_x - n_x
    start_y = center_y - n_y

    # Calculate the coordinates of the bottom-right corner of the new image
    end_x = center_x + n_x
    end_y = center_y + n_y

    # Get the section of the image as a new image
    metric_mask = mask[start_y:end_y, start_x:end_x]
    
    return metric_mask

def scale_mask(mask: np.array, out_resolution: tuple) -> np.array:
    binary_image = mask.astype('uint8')

    # Resize the image to the new resolution
    resized_image = cv2.resize(binary_image, out_resolution, interpolation=cv2.INTER_CUBIC)

    # Convert the image to integer type
    return resized_image.astype('uint8')

def mask_to_image(mask: np.ndarray, mask_values):
    out = np.zeros((mask.shape[-2], mask.shape[-1]), dtype=np.uint8)

    for i, v in enumerate(mask_values):
        out[mask == i] = v

    return out

def apply_mask_overlay(image, mask, alpha=0.3):
    # Resize the mask to the same size as the image
    mask = cv2.resize(mask, (image.shape[1], image.shape[0]))

    # Create an empty array for the orange color
    color = np.zeros_like(image, dtype=np.uint8)
    color[:] = (0, 0, 255)

    # Create a mask with the orange color where the input mask has a value of 255
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    mask_rgb = mask_rgb.astype(np.float32) / 255.0
    orange_mask = np.multiply(mask_rgb, color.astype(np.float32))

    # Apply the orange mask as a transparent overlay
    blended = cv2.addWeighted(image.astype(np.float32), 1 - alpha, orange_mask, alpha, 0)

    return blended.astype(np.uint8)

if __name__ == '__main__':

    origin = (63.441383, 10.417928) # Lat, Long
    print("Origin: ", origin)
    radius = 100 # In meters
    map_resolution = (2000, 2000)
    yaw_deg = 45
    camera_resolution = (1280, 720)
    aspect_ratio = camera_resolution[0] / camera_resolution[1]
    drone_alt = 60

    camera_hfov = 69
    camera_fov = get_fov_from_hfov(camera_resolution[0], camera_resolution[1], camera_hfov)

    ground_coverage = calculate_ground_coverage(camera_fov=camera_fov, altitude=drone_alt)

    file_path = "/home/simenallum/catkin_ws/src/msc_thesis/water_segmentation/data/trondheim_water/trondheim_water.shp"

    shapefile_gpd = read_gdp_from_file(file_path)

    large_scale_bbox_gpd = calculate_gnss_bbox(origin, radius)

    large_scale_map_gpd = calculate_map_intersections(large_scale_bbox_gpd, shapefile_gpd)

    image_mask = calculate_mask_from_gpd(large_scale_bbox_gpd, large_scale_map_gpd, map_resolution)

    rotated_image_mask = rotate_image_mask(image_mask, yaw_deg=yaw_deg)

    metric_mask = extract_metric_map(rotated_image_mask, map_resolution, radius, ground_coverage)

    scaled_mask = scale_mask(metric_mask, camera_resolution)

    plt.figure()
    plt.grid()
    plt.imshow(scaled_mask)
    plt.plot(camera_resolution[0]/2, camera_resolution[1]/2, '-o', ms=5, c='r')
    plt.show()