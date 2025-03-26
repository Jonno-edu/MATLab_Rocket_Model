import pyvista as pv
import pandas as pd
import numpy as np
import time
import math

# --- Allow Empty Meshes ---
pv.global_theme.allow_empty_mesh = True

# --- Configuration ---
CSV_FILENAME = 'rocket_trajectory_100fps.csv' # Replace with your CSV file path
ROCKET_HEIGHT = 5.0              # Visual height of the rocket in the plot
ROCKET_RADIUS = 0.5              # Visual radius of the rocket
NOZZLE_HEIGHT_FACTOR = 0.15      # How tall the nozzle is relative to rocket height
NOZZLE_RADIUS_FACTOR = 0.7       # How wide the nozzle is relative to rocket radius
THRUST_LENGTH_SCALE = 5.0        # Visual length of the thrust vector (scaled)
THRUST_COLOR = 'orange'
ROCKET_COLOR = 'silver'
NOZZLE_COLOR = 'darkgrey'         # Color for the visual nozzle
BACKGROUND_COLOR = 'black'
TRAJECTORY_COLOR = 'cyan'
GRID_COLOR = 'grey'
ANIMATION_SPEED_FACTOR = 1.0     # 1.0 for real-time, >1.0 for faster, <1.0 for slower

# --- Grid, Earth and Camera Configuration ---
GRID_BOUNDS = [-500, 500, -10, 10, -50, 2000] # Expanded Z range slightly downward for Earth
# Visual radius for the Earth sphere (adjust for visual preference)
EARTH_VISUAL_RADIUS = 1500.0
# Center the Earth so its top surface is at Z=0
EARTH_CENTER = (0.0, 0.0, -EARTH_VISUAL_RADIUS)
# Fixed offset for the camera from the rocket (when in follow mode)
CAMERA_OFFSET = np.array([-40.0, -40.0, 15.0]) # Adjusted for potentially better view

# --- Global flag for camera mode ---
follow_camera = True

# --- Helper Functions ---
def load_data(filename):
    """Loads rocket trajectory data from a CSV file."""
    try:
        df = pd.read_csv(filename)
        required_cols = ['t', 'X', 'Y', 'Z', 'theta', 'nozzleAngle']
        if not all(col in df.columns for col in required_cols):
            raise ValueError(f"CSV must contain columns: {', '.join(required_cols)}")
        print(f"Successfully loaded data from {filename}. Rows: {len(df)}")
        t = df['t'].to_numpy()
        x = df['X'].to_numpy()
        y = df['Y'].to_numpy() if 'Y' in df.columns else np.zeros_like(t)
        z = df['Z'].to_numpy()
        theta = df['theta'].to_numpy()
        nozzle_angle = df['nozzleAngle'].to_numpy()
        return t, x, y, z, theta, nozzle_angle
    except FileNotFoundError:
        print(f"Error: CSV file not found at '{filename}'")
        return None
    except Exception as e:
        print(f"Error loading CSV data: {e}")
        return None

def toggle_camera_mode():
    """Callback function to toggle camera follow mode."""
    global follow_camera
    follow_camera = not follow_camera
    mode = "Follow" if follow_camera else "Free"
    print(f"\nCamera Mode Toggled: {mode}")

# --- Main Animation Logic ---
def animate_rocket(t, x, y, z, theta, nozzle_angle):
    """Creates and runs the 3D rocket animation."""
    global follow_camera # Allow modification of the global flag
    follow_camera = True # Ensure follow mode starts enabled

    if t is None or len(t) == 0:
        print("No data to animate.")
        return

    # --- Setup the PyVista Plotter ---
    plotter = pv.Plotter(window_size=[1200, 900], lighting='three_lights')
    plotter.set_background(BACKGROUND_COLOR)

    # --- Create Rocket Body Mesh ---
    rocket_mesh = pv.Cone(center=(0, 0, ROCKET_HEIGHT / 2.0), direction=(0, 0, 1),
                          height=ROCKET_HEIGHT, radius=ROCKET_RADIUS, resolution=20)
    rocket_actor = plotter.add_mesh(rocket_mesh, color=ROCKET_COLOR, smooth_shading=True)

    # --- NEW: Create Nozzle Mesh ---
    # Positioned at the base of the rocket, pointing down initially
    nozzle_height = ROCKET_HEIGHT * NOZZLE_HEIGHT_FACTOR
    nozzle_radius = ROCKET_RADIUS * NOZZLE_RADIUS_FACTOR
    # Centered slightly below the rocket base origin (0,0,0) along its axis
    nozzle_mesh = pv.Cylinder(center=(0, 0, -nozzle_height / 2.0), direction=(0, 0, 1),
                              radius=nozzle_radius, height=nozzle_height, resolution=16, capping=True)
    nozzle_actor = plotter.add_mesh(nozzle_mesh, color=NOZZLE_COLOR)


    # --- Create Thrust Vector Mesh ---
    # Arrow starts at origin, points down initially. Length controlled by scale.
    thrust_mesh = pv.Arrow(start=(0, 0, 0), direction=(0, 0, -1), scale=THRUST_LENGTH_SCALE,
                           tip_length=0.3, tip_radius=0.15, shaft_radius=0.05)
    # Make thrust arrow start slightly below the nozzle visualization
    initial_thrust_offset = np.array([0.0, 0.0, -nozzle_height])
    thrust_actor = plotter.add_mesh(thrust_mesh, color=THRUST_COLOR)


    # --- Trajectory Path Setup ---
    trajectory_points = []
    trajectory_mesh = pv.PolyData()
    trajectory_actor = plotter.add_mesh(trajectory_mesh, color=TRAJECTORY_COLOR, line_width=3)

    # --- NEW: Create Earth Mesh ---
    try:
        earth_texture = pv.load_globe_texture()
        earth_mesh = pv.Sphere(radius=EARTH_VISUAL_RADIUS, center=EARTH_CENTER,
                               theta_resolution=60, phi_resolution=60)
        # Apply texture coordinates before adding mesh if needed (often automatic for Sphere)
        # earth_mesh.texture_map_to_sphere(inplace=True) # Might be needed in older pyvista
        plotter.add_mesh(earth_mesh, texture=earth_texture)
        print("Added textured Earth sphere.")
    except Exception as e:
        print(f"Could not load Earth texture, adding simple sphere: {e}")
        earth_mesh = pv.Sphere(radius=EARTH_VISUAL_RADIUS, center=EARTH_CENTER,
                               theta_resolution=30, phi_resolution=30)
        plotter.add_mesh(earth_mesh, color='deepskyblue')


    # --- Add Coordinate Axes and Grid ---
    plotter.show_grid(bounds=GRID_BOUNDS, color=GRID_COLOR, location='origin')
    plotter.add_axes(line_width=5, labels_off=False)
    plotter.camera.up = (0.0, 0.0, 1.0)

    # --- Initial Camera Position ---
    initial_pos = np.array([x[0], y[0], z[0]])
    plotter.camera.focal_point = initial_pos
    plotter.camera.position = initial_pos + CAMERA_OFFSET

    # --- NEW: Register Key Press Callback ---
    plotter.add_key_event('f', toggle_camera_mode) # Press 'f' to toggle follow/free cam

    print("\nStarting animation...")
    print("Press 'f' in the window to toggle camera follow mode.")
    print("Press 'q' in the window to quit.")
    plotter.show(interactive_update=True, auto_close=False)

    start_real_time = time.time()

    try:
        for i in range(len(t)):
            current_data_time = t[i]
            pos = np.array([x[i], y[i], z[i]])
            rocket_angle_rad = theta[i]     # Angle of rocket axis from Z+ (vertical)
            nozzle_dev_rad = nozzle_angle[i] # Angle of nozzle FROM rocket axis

            # --- Calculate Rocket Orientation (Rotation around World Y) ---
            # pi/2 (90deg) = vertical. 0 = horizontal west (-X)
            rocket_rot_y_rad = math.pi / 2.0 - rocket_angle_rad
            rocket_rot_y_deg = math.degrees(rocket_rot_y_rad)

            # --- Update Rocket Body Actor ---
            rocket_actor.position = pos
            rocket_actor.orientation = (0.0, rocket_rot_y_deg, 0.0) # Euler: Rx, Ry, Rz

            # --- Calculate Nozzle Orientation (Rocket Orientation + Nozzle Deviation) ---
            # Nozzle deviates by nozzle_dev_rad relative to the rocket body's axis.
            # Assuming positive nozzle_dev_rad means nozzle tilts towards world +X when rocket vertical.
            # This means rotation around the local Y axis by -nozzle_dev_rad.
            nozzle_rot_y_deg = rocket_rot_y_deg - math.degrees(nozzle_dev_rad)

            # --- Update Nozzle Actor ---
            nozzle_actor.position = pos # Nozzle base moves with rocket base
            nozzle_actor.orientation = (0.0, nozzle_rot_y_deg, 0.0)

            # --- Calculate Thrust Vector Orientation ---
            # Thrust points opposite to nozzle direction. Calculation is the same angle as nozzle.
            thrust_rot_y_deg = nozzle_rot_y_deg

            # --- Update Thrust Vector Actor ---
            # Position the *start* of the arrow relative to the rocket base, considering nozzle offset
            # We need to rotate the initial_thrust_offset by the nozzle orientation
            # Easier: just place the arrow actor's origin at the rocket base for now.
            # If more precision is needed, calculate rotated offset.
            thrust_actor.position = pos # Thrust originates near rocket base
            thrust_actor.orientation = (0.0, thrust_rot_y_deg, 0.0)


            # --- Update Trajectory Path ---
            trajectory_points.append(pos)
            if len(trajectory_points) >= 2:
                points_array = np.array(trajectory_points)
                trajectory_mesh.points = points_array
                trajectory_mesh.lines = pv.lines_from_points(points_array).lines
                trajectory_mesh.Modified() # Notify VTK of data change

            # --- Update Camera (Only if in follow mode) ---
            if follow_camera:
                plotter.camera.focal_point = pos.tolist()
                plotter.camera.position = (pos + CAMERA_OFFSET).tolist()
            # Else: User's manual camera adjustments are preserved

            # --- Update Plot and Timing ---
            plotter.update() # Render the changes

            # Real-time pacing
            if i < len(t) - 1:
                data_time_diff = t[i+1] - current_data_time
                if data_time_diff <= 0: data_time_diff = 0.001

                current_real_time = time.time()
                elapsed_real_time = current_real_time - start_real_time
                target_sim_elapsed = (current_data_time - t[0]) / ANIMATION_SPEED_FACTOR
                time_to_wait = target_sim_elapsed - elapsed_real_time

                if time_to_wait > 0:
                    time.sleep(time_to_wait)

            # Check if window closed
            if not plotter.iren or not plotter.iren.initialized:
                print("Plotter window appears to be closed.")
                break

    except KeyboardInterrupt:
        print("Animation stopped by user (Ctrl+C).")
    except Exception as e:
        print(f"An error occurred during animation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Animation finished.")
        if plotter and plotter.render_window:
             plotter.close()


# --- Script Entry Point ---
if __name__ == "__main__":
    # (Dummy data creation code remains the same - maybe add nozzle angle variation)
    import os
    if not os.path.exists(CSV_FILENAME):
        print(f"Creating dummy data file: {CSV_FILENAME}")
        dummy_data = """t,X,Y,Z,theta,nozzleAngle
0,0,0,0,1.57079,0
1,0.1,0,10,1.56,0.02
2,0.5,0,40,1.53,-0.03
3,1.5,0,90,1.48,0.01
4,3.0,0,160,1.42,0.00
5,5.0,0,250,1.35,-0.02
6,7.5,0,360,1.28,0.03
7,10.0,0,490,1.20,0.00
8,12.5,0,640,1.10,-0.01
9,15.0,0,810,1.00,0.02
10,17.5,0,1000,0.95,-0.01
11,20.0,0,1210,0.90,0.00
12,22.5,0,1440,0.85,0.01
"""
        with open(CSV_FILENAME, 'w') as f:
            f.write(dummy_data)

    data = load_data(CSV_FILENAME)
    if data:
        animate_rocket(*data)