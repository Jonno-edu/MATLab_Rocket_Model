import pandas as pd
import simplekml
import numpy as np

# Read the trajectory data
data = pd.read_csv('rocket_trajectory.csv', header=None)
# Column order is: time, x, y, z, theta
time = data[0].values
x = data[1].values
y = data[2].values
z = data[3].values  # Z is already positive in the CSV
theta = data[4].values

# Downsample the trajectory data to stay under Google Earth's vertex limit
total_points = len(time)
downsample_factor = max(1, total_points // 1000)

time = time[::downsample_factor]
x = x[::downsample_factor]
y = y[::downsample_factor]
z = z[::downsample_factor]
theta = theta[::downsample_factor]

# Launch site coordinates for reference point
LAUNCH_LAT = -34.650937  # South Africa
LAUNCH_LON = 20.216959   # South Africa

# Constants for coordinate conversion
METERS_PER_LAT = 111111  # Approximate meters per degree of latitude

# Calculate lat/lon offsets from the local coordinates
delta_lat = y / METERS_PER_LAT
delta_lon = x / (METERS_PER_LAT * np.cos(np.radians(LAUNCH_LAT)))

lat = LAUNCH_LAT + delta_lat
lon = LAUNCH_LON + delta_lon
alt = z  # Altitude is already in meters and positive up

print(f"Trajectory points reduced from {total_points} to {len(time)}")

# Create KML file with explicit style settings
kml = simplekml.Kml()

# Create a folder for the simulation with description
folder = kml.newfolder(name='Rocket Flight Simulation')
folder.description = "STEVE Rocket Flight Trajectory Visualization"

# Add launch site marker
launch_point = folder.newpoint(name="Launch Site")
launch_point.coords = [(LAUNCH_LON, LAUNCH_LAT, 0)]
launch_point.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/paddle/red-circle.png'
launch_point.style.iconstyle.scale = 1.5
launch_point.style.labelstyle.color = 'ff0000ff'
launch_point.style.labelstyle.scale = 1.2
launch_point.altitudemode = simplekml.AltitudeMode.clamptoground
launch_point.description = f"""
<![CDATA[
<h3>Launch Site Information:</h3>
<p>Latitude: {LAUNCH_LAT:.6f}</p>
<p>Longitude: {LAUNCH_LON:.6f}</p>
]]>
"""

# Define burn time (from initialize_parameters.m)
BURN_TIME = 63  # seconds

# Split trajectory into burn and coast phases
burn_mask = time <= BURN_TIME
coast_mask = time > BURN_TIME

# Create burn phase trajectory (red)
if any(burn_mask):
    burn_trajectory = folder.newlinestring(name="Burn Phase")
    burn_trajectory.coords = list(zip(lon[burn_mask], lat[burn_mask], alt[burn_mask]))
    burn_trajectory.altitudemode = simplekml.AltitudeMode.absolute
    burn_trajectory.style.linestyle.width = 3
    burn_trajectory.style.linestyle.color = 'ff0000ff'  # Red color

# Create coast phase trajectory (blue)
if any(coast_mask):
    coast_trajectory = folder.newlinestring(name="Coast Phase")
    coast_trajectory.coords = list(zip(lon[coast_mask], lat[coast_mask], alt[coast_mask]))
    coast_trajectory.altitudemode = simplekml.AltitudeMode.absolute
    coast_trajectory.style.linestyle.width = 3
    coast_trajectory.style.linestyle.color = 'ffff0000'  # Blue color

# Add flight statistics
stats_point = folder.newpoint(name="Flight Statistics")
stats_point.coords = [(LAUNCH_LON, LAUNCH_LAT, max(alt)/2)]
stats_point.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/info.png'
stats_point.style.iconstyle.scale = 1.2
stats_point.style.labelstyle.scale = 1.0
stats_point.altitudemode = simplekml.AltitudeMode.absolute
stats_point.description = f"""
<![CDATA[
<h3>Flight Statistics:</h3>
<ul>
<li>Total flight time: {time[-1]:.2f} seconds</li>
<li>Burn time: {BURN_TIME:.2f} seconds</li>
<li>Coast time: {time[-1] - BURN_TIME:.2f} seconds</li>
<li>Max altitude: {max(alt):.2f} meters</li>
<li>Ground distance traveled: {np.sqrt(x[-1]**2 + y[-1]**2):.2f} meters</li>
<li>Original data points: {total_points}</li>
<li>Simplified to: {len(time)} points</li>
</ul>
]]>
"""

# Save the KML file
kml.save("rocket_trajectory_static.kml")

print(f"""
Static KML file has been created successfully!
Launch site coordinates: {LAUNCH_LAT}, {LAUNCH_LON}
Trajectory points reduced from {total_points} to {len(time)}
Burn time: {BURN_TIME:.2f} seconds
Coast time: {time[-1] - BURN_TIME:.2f} seconds

The trajectory is now color-coded:
- Red: Burn phase (0 to {BURN_TIME:.2f} seconds)
- Blue: Coast phase ({BURN_TIME:.2f} to {time[-1]:.2f} seconds)

To view the trajectory:
1. Open Google Earth
2. File -> Open -> rocket_trajectory_static.kml
""")
