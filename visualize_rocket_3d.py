from vpython import *
import pandas as pd
import numpy as np

# Read the trajectory data saved by MATLAB
data = pd.read_csv('rocket_trajectory.csv', header=None)
# Column order is: time, x, y, z, theta
time = data[0].values
x = data[1].values
y = data[2].values
z = -data[3].values  # Negate Z values to make altitude positive
theta = data[4].values

# Calculate velocity for display
dt = time[1] - time[0]
vx = np.gradient(x, dt)
vy = np.gradient(y, dt)
vz = np.gradient(z, dt)
velocity = np.sqrt(vx**2 + vy**2 + vz**2)

# Set up the 3D visualization with an initial camera position that's more natural
scene = canvas(title='Rocket Trajectory Visualization',
              width=1200, height=800,
              background=color.black)

# Set initial camera position for a side view
scene.camera.pos = vector(200, 200, 50)
scene.camera.axis = vector(-200, -200, -50)
scene.up = vector(0, 0, 1)

# Create a detailed ground plane with grid lines
ground_size = 1000
grid_spacing = 50
ground_color = color.green
grid_color = color.gray(0.7)

# Main ground plane
ground = box(pos=vector(0,0,-1),
            size=vector(ground_size, ground_size, 0.1),
            color=ground_color,
            opacity=0.3)

# Create grid lines
for i in range(-int(ground_size/2), int(ground_size/2) + grid_spacing, grid_spacing):
    # X direction lines
    curve(pos=[vector(i, -ground_size/2, 0), vector(i, ground_size/2, 0)],
          color=grid_color, opacity=0.3)
    # Y direction lines
    curve(pos=[vector(-ground_size/2, i, 0), vector(ground_size/2, i, 0)],
          color=grid_color, opacity=0.3)

# Create a more detailed rocket
rocket_length = 30
rocket_radius = 1

# Create rocket body parts
rocket_body = compound([
    # Main body
    cylinder(pos=vector(0,0,0),
            axis=vector(rocket_length, 0, 0),
            radius=rocket_radius,
            color=color.white),
    
    # Nose cone
    cone(pos=vector(rocket_length,0,0),
         axis=vector(rocket_length/4, 0, 0),
         radius=rocket_radius,
         color=color.white),
    
    # Fins (4 fins at 90 degrees)
    box(pos=vector(rocket_length/4, 0, rocket_radius),
        size=vector(rocket_length/3, 0.2, rocket_length/6),
        color=color.white),
    box(pos=vector(rocket_length/4, 0, -rocket_radius),
        size=vector(rocket_length/3, 0.2, rocket_length/6),
        color=color.white),
    box(pos=vector(rocket_length/4, rocket_radius, 0),
        size=vector(rocket_length/3, rocket_length/6, 0.2),
        color=color.white),
    box(pos=vector(rocket_length/4, -rocket_radius, 0),
        size=vector(rocket_length/3, rocket_length/6, 0.2),
        color=color.white)
])

# Create exhaust trail effect
trail = []
trail_lifetime = 50  # How many points to keep in the trail
for _ in range(trail_lifetime):
    trail.append(sphere(radius=0.5, 
                       color=color.yellow,
                       opacity=0.0))

# Create text displays with better positioning and style
display_box = box(pos=vector(-100, 100, 100),
                 size=vector(150, 80, 1),
                 color=color.gray(0.2),
                 opacity=0.7)

time_display = label(pos=display_box.pos + vector(0, 20, 1),
                    text='Time: 0.0 s',
                    height=16,
                    color=color.cyan,
                    box=False)

altitude_display = label(pos=display_box.pos + vector(0, 0, 1),
                        text='Altitude: 0.0 m',
                        height=16,
                        color=color.cyan,
                        box=False)

velocity_display = label(pos=display_box.pos + vector(0, -20, 1),
                        text='Velocity: 0.0 m/s',
                        height=16,
                        color=color.cyan,
                        box=False)

# Animation loop
for i in range(len(time)):
    rate(1/dt)  # Control animation speed
    
    # Update rocket position and orientation
    rocket_pos = vector(x[i], y[i], z[i])
    rocket_body.pos = rocket_pos
    rocket_body.axis = vector(rocket_length*cos(theta[i]), 0, rocket_length*sin(theta[i]))
    rocket_body.rotate(angle=theta[i], axis=vector(0,0,1))
    
    # Update trail effect
    for j in range(len(trail)-1):
        trail[j].pos = trail[j+1].pos
        trail[j].opacity = max(0, trail[j].opacity - 1.0/trail_lifetime)
    
    trail[-1].pos = rocket_pos - rocket_body.axis*0.5  # Position at rocket's base
    trail[-1].opacity = 1.0
    trail[-1].color = color.orange  # Make the newest part of trail brighter
    
    # Update camera to follow rocket with a fixed offset
    offset = vector(100, 100, 50)
    scene.camera.pos = rocket_pos + offset
    scene.camera.axis = rocket_pos - scene.camera.pos
    
    # Update displays
    time_display.text = f'Time: {time[i]:.1f} s'
    altitude_display.text = f'Altitude: {z[i]:.1f} m'
    velocity_display.text = f'Velocity: {velocity[i]:.1f} m/s'
    
    # Keep display box and labels fixed relative to camera
    display_box.pos = scene.camera.pos + scene.camera.axis + vector(-100, 50, 0)
    time_display.pos = display_box.pos + vector(0, 20, 1)
    altitude_display.pos = display_box.pos + vector(0, 0, 1)
    velocity_display.pos = display_box.pos + vector(0, -20, 1)