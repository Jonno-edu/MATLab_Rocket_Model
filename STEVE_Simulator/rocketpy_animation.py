import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

# Load the CSV trajectory data
df = pd.read_csv('rocket_trajectory_60fps.csv')

# Create a figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Initialize the rocket position and orientation line
rocket_point, = ax.plot([], [], [], 'ro', markersize=8)
orientation_line, = ax.plot([], [], [], 'r-', linewidth=2)
path, = ax.plot([], [], [], 'b-', linewidth=1, alpha=0.5)

# Set axis limits based on data
max_height = df['Z'].max() * 1.1
max_range = max(abs(df['X'].max()), abs(df['X'].min())) * 1.1
if max_range < 1e-8:  # Very small values
    max_range = max_height * 0.1  # Set a reasonable default
    
ax.set_xlim(-max_range, max_range)
ax.set_ylim(-max_range, max_range)
ax.set_zlim(0, max_height)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Rocket Trajectory Animation')

# Text annotations for rocket info
time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)
altitude_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes)
attitude_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes)

# Length of attitude indicator
attitude_length = max_height * 0.05

def init():
    rocket_point.set_data([], [])
    rocket_point.set_3d_properties([])
    orientation_line.set_data([], [])
    orientation_line.set_3d_properties([])
    path.set_data([], [])
    path.set_3d_properties([])
    time_text.set_text('')
    altitude_text.set_text('')
    attitude_text.set_text('')
    return rocket_point, orientation_line, path, time_text, altitude_text, attitude_text

def animate(i):
    # Get current position
    x = df['X'][i]
    y = df['Y'][i]
    z = df['Z'][i]
    theta = df['theta'][i]
    
    # Update rocket position
    rocket_point.set_data([x], [y])
    rocket_point.set_3d_properties([z])
    
    # Compute attitude vector endpoints
    # For a rocket pointing up with theta=90° (or π/2 rad)
    dx = attitude_length * math.sin(theta - math.pi/2)
    dz = attitude_length * math.cos(theta - math.pi/2)
    
    # Update orientation line
    orientation_line.set_data([x, x + dx], [y, y])
    orientation_line.set_3d_properties([z, z + dz])
    
    # Update path trace (show complete path up to current point)
    path.set_data(df['X'][:i+1], df['Y'][:i+1])
    path.set_3d_properties(df['Z'][:i+1])
    
    # Update text information
    time_text.set_text(f'Time: {df["t"][i]:.2f} s')
    altitude_text.set_text(f'Altitude: {df["Z"][i]:.2f} m')
    attitude_text.set_text(f'Attitude: {math.degrees(df["theta"][i]):.2f}°')
    
    return rocket_point, orientation_line, path, time_text, altitude_text, attitude_text

# Create animation with 100fps for both animation and data
frames = len(df)
interval = 10  # 10ms between frames = 100fps

ani = FuncAnimation(fig, animate, frames=frames,
                    init_func=init, blit=True, interval=interval)

# Display animation (no saving)
plt.show()
