import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

# Load trajectory data
df = pd.read_csv('rocket_trajectory_100fps.csv')

# Initialize figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Rocket parameters
rocket_height = 10
rocket_radius = 0.25
max_height = df['Z'].max() * 1.1
ax.set_xlim(-max_height/2, max_height/2)
ax.set_ylim(-max_height/2, max_height/2)
ax.set_zlim(0, max_height)

# Initialize artists
path, = ax.plot([], [], [], 'b-', alpha=0.5)
rocket_artist = None

def create_cylinder(x, y, z, theta):
    # Cylinder geometry
    z_vals = np.linspace(-rocket_height/2, rocket_height/2, 50)
    theta_vals = np.linspace(0, 2*np.pi, 50)
    z_grid, theta_grid = np.meshgrid(z_vals, theta_vals)
    
    # Rotation matrix components
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    
    # Transform coordinates
    x_cyl = rocket_radius * np.cos(theta_grid)
    y_cyl = rocket_radius * np.sin(theta_grid)
    x_rot = x_cyl * cos_theta - y_cyl * sin_theta + x
    y_rot = x_cyl * sin_theta + y_cyl * cos_theta + y
    z_rot = z_grid + z  # Center vertically at trajectory point
    
    return x_rot, y_rot, z_rot

def init():
    global rocket_artist
    path.set_data([], [])
    path.set_3d_properties([])
    if rocket_artist:
        rocket_artist.remove()
    return path,

def animate(i):
    global rocket_artist
    x, y, z = df.loc[i, ['X', 'Y', 'Z']]
    theta = -df.loc[i, 'theta']
    
    # Remove previous cylinder
    if rocket_artist:
        ax.collections.remove(rocket_artist)
    
    # Create new rotated cylinder
    X, Y, Z = create_cylinder(x, y, z, theta)
    rocket_artist = ax.plot_surface(X, Y, Z, color='red', alpha=0.8)
    
    # Update path
    path.set_data(df['X'][:i+1], df['Y'][:i+1])
    path.set_3d_properties(df['Z'][:i+1])
    
    return path, rocket_artist

# Animation parameters
ani = FuncAnimation(fig, animate, frames=len(df), 
                   init_func=init, blit=False, interval=30)

plt.show()
