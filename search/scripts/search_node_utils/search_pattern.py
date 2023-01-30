import math
import pyproj
import plotly.express as px
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pymap3d


def generate_waypoints(altitude, camera_fov, overlap, area_size, lla_origin):
	print("Altitude: ", altitude)
	hfov = camera_fov[0]
	vfov = camera_fov[1]

	# Calculate coverage area for each photo
	h_coverage = (altitude * np.tan(np.deg2rad(hfov / 2))) ** 2
	v_coverage = (altitude * np.tan(np.deg2rad(vfov / 2))) ** 2

	h_coverage_overlap = (1-overlap)*h_coverage
	v_coverage_overlap = (1-overlap)*v_coverage

	grid_points = generate_sample_points((h_coverage_overlap, v_coverage_overlap), area_size)

	plot_grid_and_points((h_coverage, v_coverage), area_size, grid_points)

	LLA_points = xy_to_lat_lon_alt(grid_points, 15, lla_origin[0], lla_origin[1])

	return LLA_points

def plot_grid_and_points(grid_size, area_size, sample_points):
	fig, ax = plt.subplots()
	x_step = grid_size[0]
	y_step = grid_size[1]
	colors = np.random.rand(len(sample_points),3)
	for i, point in enumerate(sample_points):
		ax.add_patch(
			patches.Rectangle(
				(point[0] - (x_step / 2), point[1] - (y_step / 2)), x_step, y_step,
				facecolor=colors[i],
				edgecolor='black',
				alpha=0.1,
				linewidth=1
			)
		)
	ax.scatter(sample_points[:,0], sample_points[:,1], color='black')
	plt.show()


def xy_to_lat_lon_alt(xy_coordinates, altitude, local_origin_lat, local_origin_lon):
    ell_wgs84 = pymap3d.Ellipsoid('wgs84')

    # Convert the XY coordinates to longitude, latitude, and altitude
    lat_lon_alt = []
    for xy in xy_coordinates:
        lat1, lon1, h1 = pymap3d.ned2geodetic(xy[0], xy[1], altitude, \
                      local_origin_lat, local_origin_lon, altitude, \
                      ell=ell_wgs84, deg=True)  # wgs84 ellisoid
        lat_lon_alt.append((lat1, lon1, h1))
    
    return lat_lon_alt

def determine_utm_zone(longitude):
	return int((longitude + 183) / 6) + 1



def generate_sample_points(grid_size, area_size):
	x_step = area_size[0] / np.ceil(grid_size[0])
	y_step = area_size[1] / np.ceil(grid_size[1])
	x_points = np.arange(0, area_size[0], grid_size[0])
	y_points = np.arange(0, area_size[1], grid_size[1])
	
	sample_points = np.transpose([np.tile(x_points, len(y_points)), np.repeat(y_points, len(x_points))])
	return sample_points

def get_fov_from_hfov(image_width, image_height, hfov):
	aspect_ratio = image_width / image_height
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)

def plot_waypoints_on_map(list):

	df = pd.DataFrame(list, columns=["Lat", "Long", "Alt"])

	color_scale = [(0, 'orange'), (1,'red')]


	fig = px.scatter_mapbox(df, 
							lat="Lat", 
							lon="Long",
							color_continuous_scale=color_scale,
							zoom=8, 
							height=800,
							width=800)

	fig.update_layout(mapbox_style="open-street-map")
	fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
	fig.show()


def main():
	camera_fov = get_fov_from_hfov(1280, 720, 69)
	start_point = (63.504124, 10.486565, 25) # Lat, Lon, Alt
	area_size = (1000, 1000)
	overlap = 0.25
	print(camera_fov)
	waypoints = generate_waypoints(12, camera_fov, overlap, area_size, start_point)
	
	# print(waypoints)

	plot_waypoints_on_map(waypoints)

	return

if __name__ == '__main__':
	main()