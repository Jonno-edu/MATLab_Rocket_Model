import bpy
import csv
import math
from mathutils import Vector, Quaternion

# --- USER SETTINGS ---
# File path to your CSV (make sure it's the one from the corrected MATLAB script)
csv_file_path = "/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/rocket_trajectory_100fps.csv" 

# Object names in Blender
rocket_name = "RocketBody"  # Base pose: Points LOCAL Z up (nose direction)
cg_name = "CG_Empty"        # Follows world trajectory of CG
nozzle_name = "Nozzle"      # Origin at pivot point, PARENTED TO ROCKETBODY

# Rocket properties
ROCKET_LENGTH_M = 9.5418    # Total rocket length in meters

# Get the objects
try:
    rocket = bpy.data.objects[rocket_name]
    cg = bpy.data.objects[cg_name]
    nozzle = bpy.data.objects[nozzle_name]
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

print(f"Reading CSV and applying animation for {ROCKET_LENGTH_M}m rocket...")
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

        except (StopIteration, ValueError, KeyError) as e:
            print(f"Error reading header or finding required columns: {e}.\n"
                  f"Check CSV format and headers: {header}")
            header = None

        if header:
            frame = 1
            for i, row in enumerate(csv_reader):
                try:
                    # Read data using column indices
                    time_s = float(row[col_t])
                    sim_pos_x = float(row[col_px])/1
                    sim_pos_y = float(row[col_py])
                    sim_pos_z = float(row[col_pz])/1
                    sim_pitch_rad = float(row[col_pitch])
                    cg_from_nose_x = float(row[col_cg_bx])
                    cg_from_nose_y = float(row[col_cg_by])
                    cg_from_nose_z = float(row[col_cg_bz])
                    sim_nozzle_angle_rad = float(row[col_nozzle])

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
                    # Nozzle pivots around its local Y-axis as confirmed
                    nozzle.rotation_mode = 'XYZ'
                    nozzle.rotation_euler = (0, -sim_nozzle_angle_rad*10, 0)
                    nozzle.keyframe_insert(data_path="rotation_euler", frame=frame)

                    # Debug info (every 100th frame)
                    if i % 100 == 0:
                        print(f"Frame {frame}: CG at ({sim_pos_x:.2f}, {sim_pos_y:.2f}, {sim_pos_z:.2f}), "
                              f"Pitch={sim_pitch_rad*180/math.pi:.1f}°, "
                              f"CG offset from nose={cg_from_nose_x:.3f}m")

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
