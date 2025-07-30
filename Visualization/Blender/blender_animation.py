import bpy
from mathutils import Vector
import csv
import math
import os
import logging
import warnings

logging.getLogger("BlenderGIS-master").setLevel(logging.CRITICAL)
warnings.filterwarnings("ignore", message=".*'bgl' imported without an OpenGL backend.*")
blend_file_directory = os.path.dirname(bpy.data.filepath)

# --- USER SETTINGS ---
csv_file_path = os.path.join(os.path.dirname(__file__), '../../Simulator_Core/output_data/trajectory_for_blender_60fps.csv')
rocket_name = "RocketBody"
cg_name = "CG_Empty"
cp_name = "CP_Empty"
nozzle_name = "Nozzle"
trajectory_name = "RocketTrajectory"
flame_name = "Rocket Flame Diamond Shock"
camera_name = "Camera"
arrow_names = {
    "Gravity": "Arrow_Gravity",
    "Drag": "Arrow_AeroDrag",
    "Lift": "Arrow_AeroLift",
    "Thrust": "Arrow_Thrust",
}
CREATE_TRAJECTORY_CURVE = True
TRAJECTORY_COLOR = (1.0, 1.0, 0.0, 1.0)
TRAJECTORY_THICKNESS = 0.05
ROCKET_LENGTH_M = 9.5418
nozzle_angle_multiplier = 1
flame_scale_multiplier = 0.15345
aero_arrow_scale_multiplier = 1  # Scale factor for aero arrows
ARROW_FORCE_LENGTH = 5  # Max visual arrow length in meters
CAMERA_OFFSET = Vector((0, 60, 8))
# --- END USER SETTINGS ---

# Get Blender objects
try:
    rocket = bpy.data.objects[rocket_name]
    cg = bpy.data.objects[cg_name]
    cp = bpy.data.objects[cp_name]
    nozzle = bpy.data.objects[nozzle_name]
    flame = bpy.data.objects[flame_name]
    camera = bpy.data.objects.get(camera_name)
except KeyError as e:
    print(f"Error: Object '{e}' not found. Check names in Blender.")
    raise
arrow_objs = {}
for key, name in arrow_names.items():
    arrow_objs[key] = bpy.data.objects.get(name)

def clear_animation(obj):
    if obj and obj.animation_data:
        obj.animation_data_clear()

# Clear animation data for animated objects
for obj in [rocket, cg, nozzle, flame, cp, camera] + list(arrow_objs.values()):
    clear_animation(obj)

def create_trajectory_curve(points, name="RocketTrajectory"):
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name])
    curve_data = bpy.data.curves.new(name=name, type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 2
    spline = curve_data.splines.new('NURBS')
    spline.points.add(len(points)-1)
    for i, coord in enumerate(points):
        spline.points[i].co = (coord[0], coord[1], coord[2], 1.0)
    curve_obj = bpy.data.objects.new(name, curve_data)
    curve_data.bevel_depth = TRAJECTORY_THICKNESS
    curve_data.use_fill_caps = True
    mat = bpy.data.materials.new(f"{name}_material")
    mat.diffuse_color = TRAJECTORY_COLOR
    curve_obj.data.materials.append(mat) # type: ignore
    if bpy.context.collection:
        bpy.context.collection.objects.link(curve_obj)
    elif bpy.context.scene:
        bpy.context.scene.collection.objects.link(curve_obj)
    print(f"Created trajectory curve with {len(points)} points")
    return curve_obj

print(f"Reading CSV for {ROCKET_LENGTH_M}m rocket simulation...")

# ---------- CSV Read + Data Array Build ----------
data = {}
try:
    with open(csv_file_path, 'r', newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        header = next(csvreader)
        columns = {name: idx for idx, name in enumerate(header)}
        for col in header:
            data[col] = []
        all_rows = []
        trajectory_points = []
        for i, row in enumerate(csvreader):
            try:
                for col in header:
                    data[col].append(float(row[columns[col]]))
                sim_pos_x = float(row[columns['pos_x_m']])
                sim_pos_y = float(row[columns['pos_y_m']])
                sim_pos_z = float(row[columns['pos_z_m']]) + ROCKET_LENGTH_M
                trajectory_points.append((sim_pos_x, sim_pos_y, sim_pos_z))
                all_rows.append(row)
            except (ValueError, IndexError):
                continue
except FileNotFoundError as e:
    print(f"Error: File not found: {e}")
    raise
except Exception as e:
    print(f"An error occurred: {e}")
    raise
print(f"CSV columns successfully loaded: {list(data.keys())}")

frame_count = min(
    len(data['time_s']), len(data['pos_x_m']), len(data['pitch_rad']),
    len(data['cp_from_nose_m']), len(data['Fg_x_N']),
    len(data.get('thrust_N', [0]))
)

# --- Trajectory Curve Creation ---
if CREATE_TRAJECTORY_CURVE and len(trajectory_points) > 0:
    print(f"Creating trajectory curve with {len(trajectory_points)} points...")
    create_trajectory_curve(trajectory_points, trajectory_name)

# --- Assemble force magnitudes for each arrow ---
forces = {
    "Gravity": [],
    "Drag": [],
    "Lift": [],
    "Thrust": [],
}
for i in range(frame_count):
    Fg_x = data['Fg_x_N'][i]
    Fg_z = data['Fg_z_N'][i]
    forces["Gravity"].append(math.sqrt(Fg_x**2 + Fg_z**2))
    forces["Drag"].append(data.get('AxialForceAero_N', [0]*frame_count)[i])
    forces["Lift"].append(data.get('NormalForceAero_N', [0]*frame_count)[i])
    if 'thrust_N' in data:
        forces["Thrust"].append(abs(data['thrust_N'][i]))
    else:
        forces["Thrust"].append(abs(data.get('thrust_normalized', [0]*frame_count)[i]))

# --- UNIVERSAL normalization: Use thrust max as force_max for all arrows ---
force_max = max(forces["Thrust"]) if forces["Thrust"] else 1.0

print(f"Animation: All arrows normalized to the maximum thrust = {force_max:.2f} (physical units).")
last_print_time = -1.0  # Initialize to ensure the first print happens at t=0
for i in range(frame_count):
    sim_pos_x = data['pos_x_m'][i]
    sim_pos_y = data['pos_y_m'][i]
    sim_pos_z = data['pos_z_m'][i] + ROCKET_LENGTH_M
    sim_pitch_rad = data['pitch_rad'][i]
    cg_from_nose_x = ROCKET_LENGTH_M - data['cg_body_x_m'][i]
    cg_from_nose_y = data['cg_body_y_m'][i]
    cg_from_nose_z = data['cg_body_z_m'][i]
    sim_nozzle_angle_rad = data['nozzle_angle_rad'][i]
    thrust_normalized = data['thrust_normalized'][i]
    cp_from_nose = data['cp_from_nose_m'][i]
    frame = i + 1

    # 1. CG Location
    cg.location = Vector((sim_pos_x, sim_pos_y, sim_pos_z))
    cg.keyframe_insert(data_path="location", frame=frame)

    # 2. Rocket Body (corrected for CG offset)
    rocket.location = Vector(
        (sim_pos_x + cg_from_nose_x * math.cos(sim_pitch_rad),
         sim_pos_y,
         sim_pos_z + cg_from_nose_x * math.sin(sim_pitch_rad))
    )
    rocket.keyframe_insert(data_path="location", frame=frame)

    # 3. Rocket Rotation
    blender_pitch_rad = math.pi/2.0 - sim_pitch_rad
    rocket.rotation_mode = 'XYZ'
    rocket.rotation_euler = (0, blender_pitch_rad - math.pi/2, 0)
    rocket.keyframe_insert(data_path="rotation_euler", frame=frame)

    # 4. Nozzle Animation
    nozzle.location = Vector((
        sim_pos_x - (ROCKET_LENGTH_M - cg_from_nose_x) * math.cos(sim_pitch_rad),
        sim_pos_y,
        sim_pos_z - (ROCKET_LENGTH_M - cg_from_nose_x) * math.sin(sim_pitch_rad)
    ))
    nozzle.keyframe_insert(data_path="location", frame=frame)
    nozzle_global_angle_rad = sim_nozzle_angle_rad * nozzle_angle_multiplier + blender_pitch_rad
    nozzle.rotation_mode = 'XYZ'
    nozzle.rotation_euler = (0, nozzle_global_angle_rad, 0)
    nozzle.keyframe_insert(data_path="rotation_euler", frame=frame)

    # 5. Flame Animation (scale and hide)
    flame.scale = Vector((thrust_normalized * flame_scale_multiplier,)*3)
    flame.keyframe_insert(data_path="scale", frame=frame)
    flame.hide_render = (thrust_normalized < 0.01)
    flame.keyframe_insert(data_path="hide_render", frame=frame)

    # 6. CP Location RELATIVE TO CG (X-up body axis, with CORRECT sign)
    cg_val = cg_from_nose_x
    cp_val = data['cp_from_nose_m'][i]
    cg_to_cp = cg_val - cp_val
    # Print the values every 1 seconds of simulation time
    time_s = data['time_s'][i]
    if time_s >= last_print_time + 1.0:
        print(f"Time: {time_s:.2f}s | cg ({cg_val:.4f}) - cp ({cp_val:.4f}) = offset ({cg_to_cp:.4f})")
        last_print_time = time_s
    cp_offset_vec = Vector((
        cg_to_cp * math.cos(sim_pitch_rad),
        0.0,
        cg_to_cp * math.sin(sim_pitch_rad)
    ))
    cg_world = Vector((sim_pos_x, sim_pos_y, sim_pos_z))
    cp_world = cg_world + cp_offset_vec
    cp.location = cp_world
    cp.keyframe_insert(data_path="location", frame=frame)
    cp.rotation_mode = 'XYZ'
    cp.rotation_euler = (0.0, -sim_pitch_rad, 0.0)
    cp.keyframe_insert(data_path="rotation_euler", frame=frame)

    # 7. ARROW SCALE: All arrows normalized to max thrust (universal)
    for force_type, arrow_obj in arrow_objs.items():
        if arrow_obj is not None:
            force_value = forces[force_type][i]
            scale_z = (force_value / force_max) * ARROW_FORCE_LENGTH if force_max > 0 else 0.0
            
            if force_type in ["Drag", "Lift"]:
                scale_z *= -aero_arrow_scale_multiplier

            arrow_obj.scale = (1.0, 1.0, scale_z)
            arrow_obj.keyframe_insert(data_path="scale", frame=frame)

    # 8. Camera Animation
    if camera:
        camera.location = cg.location + CAMERA_OFFSET
        camera.keyframe_insert(data_path="location", frame=frame)

# Set animation endpoint
if bpy.context.scene:
    bpy.context.scene.frame_end = frame_count

print("Animation keyframes applied and force arrows (universally) normalized to thrust max.")

