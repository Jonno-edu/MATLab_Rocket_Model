import bpy
import csv
import os
import math
from mathutils import Euler, Quaternion, Vector, Matrix

print("--- Starting Blender Animation Script ---")

# --- SETTINGS ---
cg_name = "CG_Empty"
rocket_body_name = "RocketBody"
nozzle_name = "Nozzle"  # Updated to your object name

# Nozzle angle multiplier for visualization (e.g., 1.0 for actual angles, 2.0 to exaggerate)
nozzle_angle_multiplier = 2.0

# Path to the CSV file
blend_dir = os.path.dirname(bpy.data.filepath)
# Ensure this path correctly points to your CSV file
csv_file_path = os.path.join(blend_dir, "../../scripts/data/output/trajectory_blender_6dof_60fps.csv")

# --- SCENE SETUP ---
scene = bpy.context.scene
fps = scene.render.fps

# Get objects from the scene
cg = bpy.data.objects.get(cg_name)
rocket_body = bpy.data.objects.get(rocket_body_name)
nozzle = bpy.data.objects.get(nozzle_name)

# --- VALIDATE OBJECTS ---
if not all([cg, rocket_body, nozzle]):
    print("Error: One or more objects (CG, RocketBody, or Nozzle) not found. Check names in the script.")
else:
    # --- PREPARE ANIMATION DATA ---
    def setup_animation(obj, action_name):
        if not obj.animation_data:
            obj.animation_data_create()
        if not obj.animation_data.action:
            obj.animation_data.action = bpy.data.actions.new(name=action_name)
        return obj.animation_data.action

    action_cg = setup_animation(cg, "CG_Action")
    action_rocket_body = setup_animation(rocket_body, "RocketBody_Action")
    action_nozzle = setup_animation(nozzle, "Nozzle_Action")

    # Set rotation modes
    cg.rotation_mode = 'QUATERNION'
    rocket_body.rotation_mode = 'QUATERNION'
    nozzle.rotation_mode = 'XYZ'  # Use Euler for simpler local axis rotation

    # --- CLEAR EXISTING KEYFRAMES ---
    def clear_keyframes(action, data_paths):
        fcurves_to_remove = [fc for fc in action.fcurves if fc.data_path in data_paths]
        for fc in fcurves_to_remove:
            action.fcurves.remove(fc)

    clear_keyframes(action_cg, ("location", "rotation_quaternion"))
    clear_keyframes(action_rocket_body, ("location", "rotation_quaternion"))
    clear_keyframes(action_nozzle, ("rotation_euler",))

    # --- COORDINATE FRAME TRANSFORMATION MATRICES ---
    R_ned_to_enu = Matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    R_model_fix = Matrix([[0, -1, 0], [-1, 0, 0], [0, 0, 1]])

    # --- LOAD AND ANIMATE FROM CSV ---
    try:
        with open(csv_file_path, newline='') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            print(f"Loaded {len(rows)} rows from CSV at {csv_file_path}")
            
            latest_frame = 0

            for i, row in enumerate(rows):
                try:
                    # Parse time and frame
                    t = float(row["time_s"])
                    frame = round(t * fps)
                    latest_frame = max(latest_frame, frame)

                    # --- ROCKET BODY AND CG TRANSFORMATION ---
                    Xn, Xe, Xd = float(row["Xn_m"]), float(row["Xe_m"]), float(row["Xd_m"])
                    qw, qx, qy, qz = float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])
                    cg_from_tail_m = float(row["cg_from_tail_m"])

                    pos_cg = Vector((Xe, Xn, -Xd))

                    q_ned = Quaternion((qw, qx, qy, qz))
                    q_ned.normalize()
                    
                    R_body_ned = q_ned.to_matrix()
                    R_body_enu = R_ned_to_enu @ R_body_ned @ R_ned_to_enu.transposed()
                    R_body_enu_corrected = R_body_enu @ R_model_fix.transposed()
                    
                    q_enu = R_body_enu_corrected.to_quaternion()
                    q_enu.normalize()

                    local_offset = Vector((-cg_from_tail_m, 0, 0))
                    world_offset = R_body_enu_corrected @ local_offset
                    pos_tail = pos_cg - world_offset

                    # Set keyframes for CG and RocketBody
                    cg.location = pos_cg
                    cg.rotation_quaternion = q_enu
                    cg.keyframe_insert(data_path="location", frame=frame)
                    cg.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                    rocket_body.location = pos_tail
                    rocket_body.rotation_quaternion = q_enu
                    rocket_body.keyframe_insert(data_path="location", frame=frame)
                    rocket_body.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    
                    # --- NOZZLE ROTATION (UPDATED) ---
                    # Read nozzle angles directly in radians from the CSV
                    nozzle_rot_y_rad = float(row.get("y_nozzle_angle_rad", 0.0))
                    nozzle_rot_z_rad = float(row.get("z_nozzle_angle_rad", 0.0))
                    
                    # Apply multiplier for visualization
                    nozzle_rot_y_rad *= nozzle_angle_multiplier
                    nozzle_rot_z_rad *= nozzle_angle_multiplier
                    
                    # Set nozzle's local rotation using Euler angles. No conversion needed.
                    nozzle.rotation_euler = Euler((nozzle_rot_z_rad, nozzle_rot_y_rad, 0), 'XYZ')
                    nozzle.keyframe_insert(data_path="rotation_euler", frame=frame)

                    if i % max(1, len(rows) // 10) == 0:
                        print(f"[{i+1}/{len(rows)}] Processing frame {frame} (t={t:.2f}s)")

                except (ValueError, KeyError) as e:
                    print(f"Skipping row {i+1} due to error: {e}")
                    continue

            scene.frame_end = latest_frame
            bpy.context.view_layer.update()
            print("--- Animation Complete ---")

    except FileNotFoundError:
        print(f"Error: CSV file not found at {csv_file_path}")


