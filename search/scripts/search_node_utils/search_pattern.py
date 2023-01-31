import math
import pyproj
import plotly.express as px
import plotly.graph_objs as go
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pymap3d
import os


def generate_waypoints(altitude, camera_fov, overlap, area_size, lla_origin):
	hfov = camera_fov[0]
	vfov = camera_fov[1]

	# Calculate coverage area for each photo
	h_coverage = (altitude * np.tan(np.deg2rad(hfov / 2))) ** 2
	v_coverage = (altitude * np.tan(np.deg2rad(vfov / 2))) ** 2

	h_coverage_overlap = (1-overlap)*h_coverage
	v_coverage_overlap = (1-overlap)*v_coverage

	grid_points = generate_sample_points((h_coverage_overlap, v_coverage_overlap), area_size)

	# plot_grid_and_points((h_coverage, v_coverage), area_size, grid_points)

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
	for east, north in xy_coordinates:
		lat1, lon1, h1 = pymap3d.ned2geodetic(north, east, altitude, \
					  local_origin_lat, local_origin_lon, altitude, \
					  ell=ell_wgs84, deg=True)  # wgs84 ellisoid
		lat_lon_alt.append((lat1, lon1, altitude))
	
	return lat_lon_alt


def generate_sample_points(grid_size, area_size):
	x_points = np.arange(-area_size[0]/2, area_size[0]/2, grid_size[0])
	y_points = np.arange(-area_size[1]/2, area_size[1]/2, grid_size[1])
	
	sample_points = np.transpose([np.tile(x_points, len(y_points)), np.repeat(y_points, len(x_points))])

	sample_points_sorted = sort_points_by_distance_to_origin(sample_points)

	return sample_points_sorted

def sort_points_by_distance_to_origin(points):
	distances = np.linalg.norm(points, axis=1)
	sorted_indices = np.argsort(distances)
	return points[sorted_indices]

def get_fov_from_hfov(image_width, image_height, hfov):
	aspect_ratio = image_width / image_height
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)

def plot_traversal_on_map(list, distressed_coordinate=None):

    df = pd.DataFrame(list, columns=["Lat", "Long", "Alt"])

    df.reset_index(inplace=True)

    start = df.iloc[0]
    stop = df.iloc[-1]

    fig = go.Figure()

    color_scale = ['orange', 'red']

    fig.add_scattermapbox(lat=[start['Lat']],
                          lon=[start['Long']],
                          mode='markers',
                          marker=dict(size=15, color=color_scale[0]),
                          text=['Start'],
						  showlegend=True,
						  hoverinfo='text')

    fig.add_scattermapbox(lat=[stop['Lat']],
                          lon=[stop['Long']],
                          mode='markers',
                          marker=dict(size=15, color=color_scale[1]),
                          text=['Stop'],
                          hoverinfo='text')

    fig.add_scattermapbox(lat=df['Lat'],
                          lon=df['Long'],
                          mode='lines+markers',
                          line=dict(width=2, color='blue'),
                          marker=dict(size=6, color='blue'),
                          text=['' for i in range(len(df))],
                          hoverinfo='skip')

    if distressed_coordinate is not None:
        lat, long, _ = distressed_coordinate
        fig.add_scattermapbox(lat=[lat],
                              lon=[long],
                              mode='markers',
                              marker=dict(size=15, color='green'),
                              text=['Distressed'],
                              hoverinfo='lon+lat+text')

    fig.update_layout(mapbox_style="open-street-map",
                      mapbox_zoom=13,
                      mapbox_center={"lat": df['Lat'].mean(),
                                     "lon": df['Long'].mean()})
    fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
    fig.update_layout(legend=dict(x=0, y=1, traceorder="normal", font=dict(family="sans-serif", size=12, color="black"), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2))
    fig.show()
	

def closest_node(coord, coords):
	min_dist = float("inf")
	closest = None
	for c in coords:
		dist = ((c[0]-coord[0])**2 + (c[1]-coord[1])**2 + (c[2]-coord[2])**2)**0.5
		if dist < min_dist:
			closest = c
			min_dist = dist
	return closest

def traverse_closest(coords):
	path = [coords[0]]
	coords.pop(0)

	while coords:
		next_node = closest_node(path[-1], coords)
		path.append(next_node)
		coords.remove(next_node)

	return path

def traverse_coordinates(coords):
	return traverse_lines(coords, 1e-5, 1e-5)

from queue import PriorityQueue

def spiral_explore(distances):
	n = len(distances)
	m = len(distances[0])
	visited = [[False for _ in range(m)] for __ in range(n)]
	start = (n//2, m//2)
	q = PriorityQueue()
	q.put((0, start))
	while not q.empty():
		curr_dist, curr = q.get()
		if visited[curr[0]][curr[1]]:
			continue
		visited[curr[0]][curr[1]] = True
		x, y = curr
		neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
		for neighbor in neighbors:
			if 0 <= neighbor[0] < n and 0 <= neighbor[1] < m:
				q.put((curr_dist + distances[neighbor[0]][neighbor[1]], neighbor))
	return visited

def haversine(lon1, lat1, lon2, lat2):
	R = 6371  # radius of the Earth in kilometers
	dLat = math.radians(lat2 - lat1)
	dLon = math.radians(lon2 - lon1)
	lat1 = math.radians(lat1)
	lat2 = math.radians(lat2)

	a = math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2
	c = 2*math.asin(math.sqrt(a))

	return R * c

def generate_distances(coords):
	n = len(coords)
	distances = [[0] * n for _ in range(n)]
	for i in range(n):
		for j in range(i, n):
			lat1, lon1, _ = coords[i]
			lat2, lon2, _ = coords[j]
			distances[i][j] = distances[j][i] = haversine(lon1, lat1, lon2, lat2)
	return distances

def traverse_lines(gnss_points, lat_threshold, lng_threshold):
	"""
	Traverse the vertical lines in a zig-zag pattern by alternating the direction of traversal.
	Points are grouped by latitude and sorted by longitude.

	:param gnss_points: list of GNSS points as (latitude, longitude, altitude) tuples
	:param lat_threshold: maximum difference in latitude between points to be grouped together
	:param lng_threshold: maximum difference in longitude between points to be grouped together
	:return: list of GNSS points in zig-zag traversal order
	"""
	# Group points by latitude
	grouped_points = {}
	for lat, lng, alt in gnss_points:
		for group_lat, group_points in grouped_points.items():
			if abs(group_lat - lat) <= lat_threshold:
				group = group_points
				break
		else:
			grouped_points[lat] = []
			group = grouped_points[lat]
		group.append((lat, lng, alt))

	# Sort each group of points by longitude
	sorted_grouped_points = [(lat, sorted(group, key=lambda x: x[1])) for lat, group in sorted(grouped_points.items())]

	result = []
	direction = 1
	for lat, lat_points in sorted_grouped_points:
		if direction == -1:
			lat_points = lat_points[::-1]
		result.extend(lat_points)
		direction = -direction
	return result


def visualize_traversal_plt(result):
	lats, lons, alts = zip(*result)
	plt.plot(lons, lats, '-o', markersize=3)
	start = result[0]
	end = result[-1]
	plt.scatter(start[1], start[0], marker='o', color='red', label='start')
	plt.scatter(end[1], end[0], marker='x', color='green', label='end')
	plt.legend()
	plt.show()

def save_lla_to_file(lla_list, filepath, filename):
    # Check if the directory exists, if not create it
    directory = os.path.dirname(f"{filepath}/{filename}")
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    # Write the lla coordinates to the file
    with open(os.path.join(filepath, filename), 'w') as file:
        for lla in lla_list:
            file.write("{},{},{}\n".format(lla[0], lla[1], lla[2]))

def delete_first_and_return_second(filepath, filename):
    # Read the lla coordinates from the file
    lla_list = []
    with open(os.path.join(filepath, filename), 'r') as file:
        for line in file:
            lat, lon, alt = map(float, line.strip().split(','))
            lla_list.append((lat, lon, alt))

    # Delete the first coordinate and return the second coordinate
    if len(lla_list) >= 2:
        lla_list.pop(0)
        return lla_list[0]
    else:
        return []

def return_first_coordinate(filepath, filename):
    # Read the lla coordinates from the file
    lla_list = []
    with open(os.path.join(filepath, filename), 'r') as file:
        for line in file:
            lat, lon, alt = map(float, line.strip().split(','))
            lla_list.append((lat, lon, alt))

    # Return the first coordinate
    if lla_list:
        return lla_list[0]
    else:
        return []

def get_traversal_grid(altitude, camera_fov, overlap, area_size, lla_origin):
	waypoints = generate_waypoints(altitude, camera_fov, overlap, area_size, lla_origin)

	return traverse_coordinates(waypoints)

def main():
	camera_fov = get_fov_from_hfov(1280, 720, 69)
	start_point = (63.447808, 10.407823, 25) # Lat, Lon, Alt
	area_size = (250, 250)
	overlap = 0.15
	altitude = 12
	waypoints = generate_waypoints(altitude, camera_fov, overlap, area_size, start_point)

	print("Num waypoints: ", len(waypoints))

	traversed = traverse_coordinates(waypoints)

	# visualize_traversal_plt(traversed)

	plot_traversal_on_map(traversed, start_point)

	return

if __name__ == '__main__':
	main()