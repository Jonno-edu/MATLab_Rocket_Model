import bpy
import csv
import os
import math
from mathutils import Euler, Quaternion, Vector, Matrix


print("--- Starting Blender Animation Script ---")

# Settings
cg_name = "CG_Empty"
blend_dir = os.path.dirname(bpy.data.filepath)
csv_file_path = os.path.join(blend_dir, "../../scripts/data/output/trajectory_blender_6dof_60fps.csv")

# Scene setup
scene = bpy.context.scene
fps = scene.render.fps
cg = bpy.data.objects[cg_name]

# Prep animation data
if not cg.animation_data:
    cg.animation_data_create()
if not cg.animation_data.action:
    cg.animation_data.action = bpy.data.actions.new(name="CG_Action")
cg.rotation_mode = 'QUATERNION'

# Clear existing transform keyframes
if cg.animation_data.action:
    fcurves = [fc for fc in cg.animation_data.action.fcurves 
               if fc.data_path in ("location", "rotation_quaternion")]
    for fc in fcurves:
        cg.animation_data.action.fcurves.remove(fc)

# NED to ENU frame transformation
R_ned_to_enu = Matrix([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])

# Model correction:
R_model_fix = Matrix([
    [0, -1, 0],
    [-1, 0, 0],
    [0, 0, 1]
])

# Load and animate CSV
with open(csv_file_path, newline='') as f:
    reader = csv.DictReader(f)
    rows = list(reader)
    print(f"Loaded {len(rows)} rows from CSV")
    
    for i, row in enumerate(rows):
        try:
            t = float(row["time_s"])
            Xn, Xe, Xd = float(row["Xn_m"]), float(row["Xe_m"]), float(row["Xd_m"])
            qw, qx, qy, qz = float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])
            
            frame = round(t * fps)
            
            # Position: NED(N,E,D) -> ENU(E,N,-D)
            pos = Vector((Xe, Xn, -Xd))
            
            # Orientation: Convert q_be from NED to ENU frame
            q_ned = Quaternion((qw, qx, qy, qz))
            q_ned.normalize()
            
            # Convert quaternion to rotation matrix
            R_body_ned = q_ned.to_matrix()
            
            # Transform to ENU frame
            R_body_enu = R_ned_to_enu @ R_body_ned @ R_ned_to_enu.transposed()
            
            # Apply model correction
            R_body_enu_corrected = R_body_enu @ R_model_fix.transposed()
            
            # Convert back to quaternion
            q_enu = R_body_enu_corrected.to_quaternion()
            q_enu.normalize()
            
            # Set keyframes
            cg.location = pos
            cg.rotation_quaternion = q_enu
            cg.keyframe_insert(data_path="location", frame=frame)
            cg.keyframe_insert(data_path="rotation_quaternion", frame=frame)
            
            if i % max(1, len(rows) // 10) == 0:
                print(f"[{i+1}/{len(rows)}] t={t:.3f}s frame={frame}")
                
        except (ValueError, KeyError) as e:
            print(f"Row {i} error: {e}")
            continue

scene.frame_end = max(scene.frame_end, frame)
bpy.context.view_layer.update()
print("--- Animation Complete ---")
