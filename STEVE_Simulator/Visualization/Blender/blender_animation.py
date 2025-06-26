import bpy
import csv
import math
from mathutils import Vector, Quaternion

# --- USER SETTINGS ---
# File path to your CSV (make sure it's the one from the corrected MATLAB script)
csv_file_path = "/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/output_data/trajectory_for_blender_60fps.csv"
blend_file_path = "/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Visualization/Blender/RocketAnimation.blend"
# Object names in Blender
rocket_name = "RocketBody"  # Base pose: Points LOCAL Z up (nose direction)
cg_name = "CG_Empty"        # Follows world trajectory of CG
nozzle_name = "Nozzle"      # Origin at pivot point, PARENTED TO ROCKETBODY
trajectory_name = "RocketTrajectory"  # Name for the static trajectory curve
flame_name = "Rocket Flame Diamond Shock" # Name of the flame object to scale

# Trajectory visualization settings
CREATE_TRAJECTORY_CURVE = True       # Set to True to create static trajectory spline
TRAJECTORY_COLOR = (1.0, 1.0, 0.0, 1.0)  # Bright light yellow (R,G,B,A)
# Or using hex conversion:
# HEX: #FFFF33 converted to RGBA (1.0, 1.0, 0.2, 1.0
TRAJECTORY_THICKNESS = 0.05          # Thickness of the trajectory curve

# Rocket properties
ROCKET_LENGTH_M = 9.5418    # Total rocket length in meters
nozzle_angle_multiplier = 5  # Multiplier for nozzle angle (if needed)
flame_scale_multiplier = 0.15345 # Multiplier for flame scale (if needed)
# --- END USER SETTINGS ---

# Get the objects
try:
    rocket = bpy.data.objects[rocket_name]
    cg = bpy.data.objects[cg_name]
    nozzle = bpy.data.objects[nozzle_name]
    flame = bpy.data.objects[flame_name] # Get the flame object
except KeyError as e:
    print(f"Error: Object '{e}' not found. Check names in Blender.")
    raise # Stop script

# Function to clear existing animation data
def clear_animation(obj):
    if obj and obj.animation_data:
        obj.animation_data_clear()

# Clear existing animation data
clear_animation(rocket)
clear_animation(cg)
clear_animation(nozzle)
clear_animation(flame) # Clear flame animation

# Function to create a trajectory curve from points
def create_trajectory_curve(points, name="RocketTrajectory"):
    # Remove existing trajectory curve if it exists
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name])
    
    # Create the curve data
    curve_data = bpy.data.curves.new(name=name, type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 2  # Smoothness of curve
    
    # Create the trajectory spline
    spline = curve_data.splines.new('NURBS')
    spline.points.add(len(points)-1)  # Add the required number of points
    
    # Set the coordinates of each point
    for i, coord in enumerate(points):
        spline.points[i].co = (coord[0], coord[1], coord[2], 1.0)  # w=1.0 for NURBS
    
    # Create the object with the curve data
    curve_obj = bpy.data.objects.new(name, curve_data)
    
    # Set curve properties
    curve_data.bevel_depth = TRAJECTORY_THICKNESS  # Thickness
    curve_data.use_fill_caps = True
    
    # Add material to the curve
    mat = bpy.data.materials.new(f"{name}_material")
    mat.diffuse_color = TRAJECTORY_COLOR  # Cyan color
    curve_obj.data.materials.append(mat)
    
    # Link the curve to the scene
    bpy.context.collection.objects.link(curve_obj)
    print(f"Created trajectory curve with {len(points)} points")
    
    return curve_obj

print(f"Reading CSV for {ROCKET_LENGTH_M}m rocket simulation...")
try:
    with open(csv_file_path, 'r') as file:
        csv_reader = csv.reader(file)
        try:
            header = next(csv_reader)
            print(f"CSV Header: {header}")
            # Find column indices from header
            col_t = header.index('time_s')
            col_px = header.index('pos_x_m')
            col_py = header.index('pos_y_m')
            col_pz = header.index('pos_z_m')
            col_pitch = header.index('pitch_rad')
            col_cg_bx = header.index('cg_body_x_m')
            col_cg_by = header.index('cg_body_y_m')
            col_cg_bz = header.index('cg_body_z_m')
            col_nozzle = header.index('nozzle_angle_rad')
            col_thrust_norm = header.index('thrust_normalized')

        except (StopIteration, ValueError, KeyError) as e:
            print(f"Error reading header or finding required columns: {e}.\n"
                  f"Check CSV format and headers: {header}")
            header = None

        if header:
            # First, read all trajectory points
            trajectory_points = []
            all_rows = []
            
            # Make a first pass through the file to collect trajectory points
            for i, row in enumerate(csv_reader):
                try:
                    # Read position data
                    time_s = float(row[col_t])
                    sim_pos_x = float(row[col_px])
                    sim_pos_y = float(row[col_py])
                    sim_pos_z = float(row[col_pz]) + ROCKET_LENGTH_M
                    
                    # Store trajectory point
                    trajectory_points.append((sim_pos_x, sim_pos_y, sim_pos_z))
                    
                    # Store entire row for later animation
                    all_rows.append(row)
                    
                except (ValueError, IndexError) as e:
                    print(f"Error reading trajectory point in row {i+2}: {e}. Skipping.")
                    continue
            
            # Create trajectory curve if enabled
            if CREATE_TRAJECTORY_CURVE and trajectory_points:
                print(f"Creating trajectory curve with {len(trajectory_points)} points...")
                trajectory_curve = create_trajectory_curve(trajectory_points, trajectory_name)
            
            # Now process animation frames
            print("Applying animation keyframes...")
            frame = 1
            for i, row in enumerate(all_rows):
                try:
                    # Read data using column indices
                    time_s = float(row[col_t])
                    sim_pos_x = float(row[col_px])
                    sim_pos_y = float(row[col_py])
                    sim_pos_z = float(row[col_pz]) + ROCKET_LENGTH_M
                    sim_pitch_rad = float(row[col_pitch])
                    cg_from_nose_x = ROCKET_LENGTH_M - float(row[col_cg_bx])
                    cg_from_nose_y = float(row[col_cg_by])
                    cg_from_nose_z = float(row[col_cg_bz])
                    sim_nozzle_angle_rad = float(row[col_nozzle])
                    thrust_normalized = float(row[col_thrust_norm]) # Read normalized thrust

                    # --- Apply Transformations ---

                    # 1. Set CG Empty Location (World Space)
                    cg.location = Vector((sim_pos_x, sim_pos_y, sim_pos_z))
                    cg.keyframe_insert(data_path="location", frame=frame)

                    # 2. Set Rocket Body LOCAL Position (corrected)
                    # Since the origin is at the nose and CG is below the nose inside the rocket,
                    # we need to offset the nose negative along local Z (because Z points out of nose)
                    rocket.location = Vector((sim_pos_x + cg_from_nose_x * math.cos(sim_pitch_rad), sim_pos_y, sim_pos_z + cg_from_nose_x * math.sin(sim_pitch_rad)))  # Negative Z offset
                    rocket.keyframe_insert(data_path="location", frame=frame)

                    # 3. Apply rotation around the Y axis (for pitching)
                    # In simulation: theta = 90° (pi/2) is vertical (up), theta = 0° is horizontal
                    # When the rocket is at 0,0,0 rotation in Blender, it's pointing straight up (local Z)
                    blender_pitch_rad = math.pi/2.0 - sim_pitch_rad # Correct calculation
                    
                    rocket.rotation_mode = 'XYZ'
                    rocket.rotation_euler = (0, blender_pitch_rad, 0)  # Y-axis rotation
                    rocket.keyframe_insert(data_path="rotation_euler", frame=frame)

                    # 4. Set Nozzle LOCAL Rotation (Relative to RocketBody parent)
                    nozzle.location = Vector((sim_pos_x - (ROCKET_LENGTH_M - cg_from_nose_x) * math.cos(sim_pitch_rad), sim_pos_y, sim_pos_z - (ROCKET_LENGTH_M - cg_from_nose_x) * math.sin(sim_pitch_rad)))
                    nozzle.keyframe_insert(data_path="location", frame=frame)

                    # Nozzle pivots around its local Y-axis as confirmed
                    nozzle_global_angle_rad = sim_nozzle_angle_rad * nozzle_angle_multiplier + blender_pitch_rad
                    nozzle.rotation_mode = 'XYZ'
                    nozzle.rotation_euler = (0, nozzle_global_angle_rad, 0)
                    nozzle.keyframe_insert(data_path="rotation_euler", frame=frame)
                    
                    # 5. Scale Flame based on Normalized Thrust
                    # Assuming flame's default scale is (1,1,1) and it points down Z
                    # We scale the Z axis based on thrust. Adjust axis if needed.
                    flame.scale = (flame_scale_multiplier, flame_scale_multiplier*thrust_normalized, flame_scale_multiplier) 
                    flame.keyframe_insert(data_path="scale", frame=frame)

                    # Debug info (every 100th frame)
                    if i % 1000 == 0:
                        print(f"Frame {frame}: CG at ({sim_pos_x:.2f}, {sim_pos_y:.2f}, {sim_pos_z:.2f}), "
                              f"Pitch={sim_pitch_rad*180/math.pi:.1f}°, "
                              f"CG offset from nose={cg_from_nose_x:.3f}m, "
                              f"ThrustNorm={thrust_normalized:.2f}") # Added thrust to debug

                    frame += 1

                except (ValueError, IndexError) as e:
                    print(f"Error processing row {i+2}: {row}. Error: {e}. Skipping.")
                    continue

            if frame > 1:
                bpy.context.scene.frame_end = frame - 1
            print(f"Animation complete. Created {frame-1} frames.")

except FileNotFoundError:
    print(f"Error: CSV file not found at '{csv_file_path}'")
except Exception as e:
    print(f"An unexpected error occurred: {e}")