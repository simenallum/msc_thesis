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
    print("Bounding box: ", west, south, east, north)

    # Create a polygon geometry from the bounding box coordinates
    bbox_polygon = Polygon([(west, north), (east, north), (east, south), (west, south)])

    large_scale_bbx_gpd = gpd.GeoDataFrame(geometry=[bbox_polygon], crs="EPSG:4326")

    return large_scale_bbx_gpd, bounds

def calculate_map_intersections(bbox: gpd.GeoDataFrame, multipolygon_gpd: gpd.GeoDataFrame, bounds) -> gpd.GeoDataFrame:

    # read the first GeoDataFrame with a single polygon
    df1 = bbox

    # read the second GeoDataFrame with multiple polygons
    df2 = multipolygon_gpd

    # extract the polygon from df1 (assuming it's the only one)
    polygon1 = df1.geometry[0]

    # create a spatial index for df2
    sindex = df2.sindex

    # find the polygons in df2 that intersect with polygon1
    possible_matches_index = list(sindex.intersection(polygon1.bounds))
    possible_matches = df2.iloc[possible_matches_index].loc[df2.intersects(polygon1)]

    # compute the intersection between polygon1 and all polygons in possible_matches
    intersections = possible_matches.intersection(polygon1)

    # compute the union of the resulting polygons
    new_polygon = unary_union(intersections)

    print("polygon from intersection: ", new_polygon.bounds)

    north, south, east, west = bounds

    # create a bounding box polygon
    bbox = Polygon([(west, north), (east, north), (east, south), (west, south)])

    # buffer the bounding box polygon
    buffered_bbox = bbox.buffer(1e-10)

    # create a new GeoDataFrame with the new polygon
    large_scale_map_gpd = gpd.GeoDataFrame({'geometry': [buffered_bbox]}, crs=df1.crs)
    poly = gpd.GeoDataFrame({'geometry': [new_polygon]}, crs=df1.crs)

    return large_scale_map_gpd, poly



def plot_gpf_frame(frame: gpd.GeoDataFrame) -> None:
    frame.plot()
    plt.grid()
    plt.show()

def calculate_mask_from_gpd(frame: gpd.GeoDataFrame, poly, mask_resolution: tuple = (2000, 2000)) -> np.array:

    poly = poly.geometry[0]

    # Load GeoPandas DataFrame containing the polygon
    gdf = frame

    # Extract the polygon from the GeoPandas DataFrame
    polygon = gdf.geometry[0]

    # Define the resolution of the mask
    mask_width, mask_height = mask_resolution

    # Define the bounding box of the mask (in the same CRS as the polygon)
    bounds = polygon.bounds
    xmin, ymin, xmax, ymax = bounds
    print("In mask creation: ", bounds)
    xres = (xmax - xmin) / mask_width
    yres = (ymax - ymin) / mask_height

    # Create an empty numpy array to store the mask
    mask = np.zeros((mask_height, mask_width), dtype=np.uint8)

    # Convert the polygon to a binary mask
    mask_img = geometry_mask(geometries=[poly],
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


if __name__ == '__main__':

    origin = (63.441347, 10.418278) # Lat, Long
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

    large_scale_bbox_gpd, bounds = calculate_gnss_bbox(origin, radius)

    large_scale_map_gpd, poly = calculate_map_intersections(large_scale_bbox_gpd, shapefile_gpd, bounds)

    large_scale_map_gpd.plot()
    plt.xlim([bounds[3], bounds[2]])
    plt.ylim([bounds[1], bounds[0]])
    plt.grid()
    plt.show()
    

    plot_gpf_frame(large_scale_map_gpd)

    image_mask = calculate_mask_from_gpd(large_scale_map_gpd, poly, map_resolution)

    rotated_image_mask = rotate_image_mask(image_mask, yaw_deg=yaw_deg)

    metric_mask = extract_metric_map(rotated_image_mask, map_resolution, radius, ground_coverage)

    scaled_mask = scale_mask(metric_mask, camera_resolution)

    plt.figure()
    plt.grid()
    plt.imshow(scaled_mask)
    plt.show()