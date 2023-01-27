import math
from pyproj import Proj, transform
import plotly.express as px
import pandas as pd


def generate_waypoints(start_pos, width, height, overlap, hfov_deg, altitude, focal_length, image_height, image_width):
    # Define the UTM zone for the starting position
    zone = int((start_pos[0] + 180) / 6) + 1
    proj_in = Proj(proj='latlong', datum='WGS84')
    proj_out = Proj(proj='utm', zone=zone, datum='WGS84')

    # Convert the starting position to UTM
    easting, northing = transform(proj_in, proj_out, start_pos[0], start_pos[1])

    #Convert FOV from degrees to radians
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.atan(math.tan(hfov_rad / 2) * image_height / image_width)
    #Convert FOV from radians to meters
    fov_m = 2 * (altitude * math.tan(vfov_rad / 2))
    #Calculate the size of each grid cell based on the camera FOV and overlap
    cell_width = (fov_m / focal_length) * (1 - overlap)
    cell_height = (fov_m / focal_length) * (1 - overlap)

    #Calculate the number of columns and rows in the grid
    cols = math.ceil(width / cell_width)
    rows = math.ceil(height / cell_height)

    # Initialize list to store the GNSS positions
    waypoints = []
    
    # Set the easting and northing to the center of the search area
    easting = easting - (width / 2)
    northing = northing - (height / 2)

    # Iterate through each column and row to calculate the GNSS position for each grid cell
    for i in range(cols):
        for j in range(rows):
            # Calculate the easting and northing for the current grid cell
            easting_ = easting + (i * cell_width)
            northing_ = northing + (j * cell_height)

            # Convert the UTM position back to latitude and longitude
            lon, lat = transform(proj_out, proj_in, easting_, northing_)

			 # Append the GNSS position as a tuple to the list of waypoints
            waypoints.append((lon, lat, altitude))
    return waypoints


def get_fov_from_hfov(image_width, image_height, hfov):
	aspect_ratio = image_height / image_width
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)

def plot_waypoints_on_map(list):

	df = pd.DataFrame(list, columns=["Long", "Lat", "Alt"])

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
	fov = get_fov_from_hfov(1280, 720, 69)
	waypoints = generate_waypoints((63.504124, 10.486565), 1000, 1000, 0.1, 69, 15, 4.73e-3, 720, 1280)
	
	print(fov)
	print(waypoints)

	plot_waypoints_on_map(waypoints)

	return

if __name__ == '__main__':
	main()