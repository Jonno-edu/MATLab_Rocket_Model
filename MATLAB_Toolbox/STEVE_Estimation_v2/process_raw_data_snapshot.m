%% 3-Axis Attitude from Reference Data Snapshot
% This script uses the REFERENCE velocity from two lines of CSV data to
% verify the TRIAD algorithm with a "perfect" GPS signal.

clear; clc; close all;

%% 1. Parse Raw Data from new CSV lines
line1 = [73327, 7672.3188, 0.0177, -39519.3590, 323.7892, 0.0006, -1702.0229, ...
         0.7659, -4.98e-10, 0.6428, -5.99e-10, 7668.7572, 0.0177, -39500.6367, ...
         323.7767, 0.0006, -1702.0417, -7.9277, 0.2010, 0.4811, ...
         0.0994, 0.0994, 0.0994, 0, 26.98, 0];

line2 = [73328, 7672.6426, 0.0177, -39521.0611, 323.7903, 0.0006, -1702.0211, ...
         0.7659, -4.98e-10, 0.6428, -6.00e-10, 7668.7572, 0.0177, -39500.6367, ...
         323.7767, 0.0006, -1702.0417, -7.9205, 0.2010, 0.4811, ...
         0.0994, 0.0994, 0.0994, 0, 26.98, 0];

% Define constants
g0 = 9.80665;

%% 2. Calculate 'a_inertial_NED' from REFERENCE Velocity

% Extract time and REFERENCE velocities
t1 = line1(1) / 1000; % Convert ms to seconds
t2 = line2(1) / 1000;
dt = t2 - t1;

% **MODIFICATION HERE: Using columns 5, 6, 7 for ref_Ve**
v_ref_1 = [line1(5), line1(6), line1(7)]'; % [North, East, Down]
v_ref_2 = [line2(5), line2(6), line2(7)]';

% Calculate the derivative using finite difference
a_inertial_NED = (v_ref_2 - v_ref_1) / dt;

%% 3. Construct Reference and Observation Vectors

% --- Reference Vectors (in the NED frame) ---

% Vector 1: The 'sensed acceleration' (Specific Force)
a_gravity_NED = [0; 0; g0];
ref_vector_1_NED = a_inertial_NED - a_gravity_NED;

% Vector 2: The magnetic field reference vector.
ref_vector_2_NED = [0; 27; 0];

% --- Observation Vectors (from sensors at time t2) ---

% Vector 1: The direct reading from the onboard accelerometer at t2
obs_vector_1_body = [line2(18); line2(19); line2(20)];

% Vector 2: The reading from the onboard magnetometer at t2
obs_vector_2_body = [line2(24); line2(25); line2(26)];

fprintf('--- Calculated & Observed Vectors (Using Reference Velocity) ---\n');
fprintf('Time Step (dt): %.4f s\n', dt);
fprintf('Ref-Derived Inertial Accel (NED): [%.2f, %.2f, %.2f]\n', a_inertial_NED);
fprintf('Ref Sensed Accel (NED):           [%.2f, %.2f, %.2f]\n', ref_vector_1_NED);
fprintf('Obs Sensed Accel (Body):          [%.2f, %.2f, %.2f]\n', obs_vector_1_body);
fprintf('Ref Mag (NED):                    [%.2f, %.2f, %.2f]\n', ref_vector_2_NED);
fprintf('Obs Mag (Body):                   [%.2f, %.2f, %.2f]\n\n', obs_vector_2_body);

%% 4. Implement TRIAD Algorithm to Find Attitude at Time t2

% Normalize vectors
r1 = ref_vector_1_NED / norm(ref_vector_1_NED);
b1 = obs_vector_1_body / norm(obs_vector_1_body);
r2 = ref_vector_2_NED / norm(ref_vector_2_NED);
b2 = obs_vector_2_body / norm(obs_vector_2_body);

% Create the TRIAD coordinate systems
t1_ref = r1; t1_obs = b1;
t2_ref = cross(r1, r2) / norm(cross(r1, r2));
t2_obs = cross(b1, b2) / norm(cross(b1, b2));
t3_ref = cross(t1_ref, t2_ref);
t3_obs = cross(t1_obs, t2_obs);

% Construct attitude matrices
M_ref = [t1_ref'; t2_ref'; t3_ref'];
M_obs = [t1_obs'; t2_obs'; t3_obs'];

% Calculate the DCM from Body to NED
R_est_body_to_ned = M_ref' * M_obs;

%% 5. Display the Results using Quaternion Error

% --- Ground Truth Attitude ---
true_quat_ZYX = [line2(8), line2(9), line2(10), line2(11)]; % [w, x, y, z]
true_eul_rad = quat2eul(true_quat_ZYX, 'ZYX');
true_yaw   = rad2deg(true_eul_rad(1));
true_pitch = rad2deg(true_eul_rad(2));
true_roll  = rad2deg(true_eul_rad(3));

fprintf('--- Ground Truth Attitude at t=%.3f s ---\n', t2);
fprintf('True Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n', true_roll, true_pitch, true_yaw);
fprintf('True Quaternion (w,x,y,z): [%.4f, %.4f, %.4f, %.4f]\n\n', true_quat_ZYX);

% --- Estimated Attitude ---
% First, convert the estimated rotation matrix to a quaternion
est_quat_ZYX = rotm2quat(R_est_body_to_ned');

% Then convert to Euler angles just for display
eul_est_rad = quat2eul(est_quat_ZYX, 'ZYX');
est_yaw   = rad2deg(eul_est_rad(1));
est_pitch = rad2deg(eul_est_rad(2));
est_roll  = rad2deg(eul_est_rad(3));

fprintf('--- TRIAD Algorithm Attitude Estimate at t=%.3f s ---\n', t2);
fprintf('Estimated Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n', est_roll, est_pitch, est_yaw);
fprintf('Estimated Quaternion (w,x,y,z): [%.4f, %.4f, %.4f, %.4f]\n\n', est_quat_ZYX);


%% 6. Calculate and Display the Quaternion Error

% The error quaternion q_err is the rotation that transforms the estimated
% attitude to the true attitude.
% q_err = q_true * quatinv(q_est)
% Note: MATLAB's quatmultiply is often more stable. q_err = quatmultiply(q_true, quatinv(q_est))

% Ensure quaternions are normalized
q_true = true_quat_ZYX / norm(true_quat_ZYX);
q_est = est_quat_ZYX / norm(est_quat_ZYX);

% Calculate the error quaternion
q_err = quatmultiply(q_true, quatinv(q_est));

% Extract the angle and axis of the error rotation
% Angle (theta) = 2 * acos(q_err_w)
error_angle_rad = 2 * acos(q_err(1));
error_angle_deg = rad2deg(error_angle_rad);

% Axis (k) = [q_err_x, q_err_y, q_err_z] / sin(theta/2)
% Avoid division by zero if angle is very small
if abs(sin(error_angle_rad / 2)) < 1e-9
    error_axis = [0, 0, 0];
else
    error_axis = [q_err(2), q_err(3), q_err(4)] / sin(error_angle_rad / 2);
end

fprintf('--- Quaternion Error Analysis ---\n');
fprintf('Total Error Angle: %.4f degrees\n', error_angle_deg);
fprintf('Error occurs around axis (x,y,z): [%.4f, %.4f, %.4f]\n', error_axis);

% For context, show the simple Euler angle subtraction as well
error_roll_eul  = est_roll - true_roll;
error_pitch_eul = est_pitch - true_pitch;
error_yaw_eul   = est_yaw - true_yaw;
fprintf('\n(For Comparison) Euler Angle Subtraction Error:\n');
fprintf('Roll: %.4f deg, Pitch: %.4f deg, Yaw: %.4f deg\n', error_roll_eul, error_pitch_eul, error_yaw_eul);
