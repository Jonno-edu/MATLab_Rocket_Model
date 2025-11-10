%% Full Flight Attitude Estimation
% Estimate attitude using TRIAD for the entire flight trajectory
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end
g0 = 9.80665;

%% Sample every 20th point (starting from index 2)
sample_indices = 2:20:size(data, 1);
n_samples = length(sample_indices);

% Pre-allocate arrays
time_s = zeros(n_samples, 1);
true_roll = zeros(n_samples, 1);
true_pitch = zeros(n_samples, 1);
true_yaw = zeros(n_samples, 1);
est_roll = zeros(n_samples, 1);
est_pitch = zeros(n_samples, 1);
est_yaw = zeros(n_samples, 1);
error_angle = zeros(n_samples, 1);

%% Main Loop
fprintf('Processing %d samples...\n', n_samples);
for i = 1:n_samples
    index = sample_indices(i);
    
    % Get data
    line1 = data(index-1, :);
    line2 = data(index, :);
    dt = (line2.time_ms - line1.time_ms) / 1000;
    if dt == 0, dt = 0.001; end
    
    time_s(i) = line2.time_ms / 1000;
    
    % Calculate inertial acceleration in NED
    v_ref_1 = [line1.ref_Ve_North, line1.ref_Ve_East, line1.ref_Ve_Down]';
    v_ref_2 = [line2.ref_Ve_North, line2.ref_Ve_East, line2.ref_Ve_Down]';
    a_inertial_NED = (v_ref_2 - v_ref_1) / dt;
    
    % Reference and observation vectors
    a_gravity_NED = [0; 0; g0];
    ref_vector_1_NED = a_inertial_NED - a_gravity_NED;
    ref_vector_2_NED = [0; 27; 0];
    
    obs_vector_1_body = -[line2.accel_x; line2.accel_y; line2.accel_z];
    obs_vector_2_body = [line2.mag_x; line2.mag_y; line2.mag_z];
    
    % TRIAD attitude solution
    B = obs_vector_1_body * ref_vector_1_NED' + obs_vector_2_body * ref_vector_2_NED';
    [U, ~, V] = svd(B);
    M = diag([1, 1, det(U) * det(V)]);
    R_est_body_to_ned = V * M * U';
    
    % Ground truth
    true_quat = [line2.ref_qw, line2.ref_qx, line2.ref_qy, line2.ref_qz];
    true_eul = rad2deg(quat2eul(true_quat, 'ZYX'));
    true_roll(i) = true_eul(3);
    true_pitch(i) = true_eul(2);
    true_yaw(i) = true_eul(1);
    
    % Estimated attitude
    est_eul = rad2deg(rotm2eul(R_est_body_to_ned, 'ZYX'));
    est_roll(i) = est_eul(3);
    est_pitch(i) = est_eul(2);
    est_yaw(i) = est_eul(1);
    
    % Error calculation
    est_quat_from_R = rotm2quat(R_est_body_to_ned);
    q_err = quatmultiply(true_quat, quatinv(est_quat_from_R));
    error_angle_rad = 2 * acos(q_err(1));
    if error_angle_rad > pi
        error_angle_rad = 2*pi - error_angle_rad;
    end
    error_angle(i) = rad2deg(error_angle_rad);
    
    if mod(i, 50) == 0
        fprintf('Processed %d/%d samples (%.1f%%)\n', i, n_samples, 100*i/n_samples);
    end
end

fprintf('Processing complete!\n\n');

%% Plotting

% Roll comparison
figure('Position', [100, 100, 1200, 800]);
subplot(4,1,1);
plot(time_s, true_roll, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, est_roll, 'r--', 'LineWidth', 1.5);
ylabel('Roll (deg)');
legend('Ground Truth', 'TRIAD Estimate', 'Location', 'best');
grid on;
title('Attitude Estimation - Full Flight');

% Pitch comparison
subplot(4,1,2);
plot(time_s, true_pitch, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, est_pitch, 'r--', 'LineWidth', 1.5);
ylabel('Pitch (deg)');
legend('Ground Truth', 'TRIAD Estimate', 'Location', 'best');
grid on;

% Yaw comparison
subplot(4,1,3);
plot(time_s, true_yaw, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, est_yaw, 'r--', 'LineWidth', 1.5);
ylabel('Yaw (deg)');
legend('Ground Truth', 'TRIAD Estimate', 'Location', 'best');
grid on;

% Total error angle
subplot(4,1,4);
plot(time_s, error_angle, 'k-', 'LineWidth', 1.5);
ylabel('Total Error (deg)');
xlabel('Time (s)');
grid on;
legend('Quaternion Error Angle', 'Location', 'best');

%% Statistics
fprintf('=== ATTITUDE ESTIMATION STATISTICS ===\n');
fprintf('Roll  - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(est_roll - true_roll)), std(est_roll - true_roll), max(abs(est_roll - true_roll)));
fprintf('Pitch - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(est_pitch - true_pitch)), std(est_pitch - true_pitch), max(abs(est_pitch - true_pitch)));
fprintf('Yaw   - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(est_yaw - true_yaw)), std(est_yaw - true_yaw), max(abs(est_yaw - true_yaw)));
fprintf('Total - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(error_angle), std(error_angle), max(error_angle));

%% Detailed Debug Analysis at 69s and 75s
fprintf('\n\n');
fprintf('################################################################\n');
fprintf('### DETAILED DEBUG ANALYSIS AT 69s AND 75s ###\n');
fprintf('################################################################\n\n');

% Find indices closest to 69s and 75s
[~, idx_69s] = min(abs([data.time_ms]/1000 - 69));
[~, idx_75s] = min(abs([data.time_ms]/1000 - 75));

% Make sure indices are valid (>1)
if idx_69s < 2, idx_69s = 2; end
if idx_75s < 2, idx_75s = 2; end

debug_times = [69, 75];
debug_indices = [idx_69s, idx_75s];

for i = 1:length(debug_indices)
    index = debug_indices(i);
    target_time = debug_times(i);
    
    fprintf('============================================================\n');
    fprintf('DETAILED ANALYSIS AT t = %d seconds (index %d)\n', target_time, index);
    fprintf('============================================================\n\n');
    
    % Get data
    line1 = data(index-1, :);
    line2 = data(index, :);
    dt = (line2.time_ms - line1.time_ms) / 1000;
    if dt == 0, dt = 0.001; end
    
    fprintf('Actual time: %.3f s\n', line2.time_ms/1000);
    fprintf('dt: %.6f s\n\n', dt);
    
    % Calculate inertial acceleration
    v_ref_1 = [line1.ref_Ve_North, line1.ref_Ve_East, line1.ref_Ve_Down]';
    v_ref_2 = [line2.ref_Ve_North, line2.ref_Ve_East, line2.ref_Ve_Down]';
    a_inertial_NED = (v_ref_2 - v_ref_1) / dt;
    
    fprintf('--- Velocity Analysis ---\n');
    fprintf('v1 (NED): [%.2f, %.2f, %.2f] m/s\n', v_ref_1);
    fprintf('v2 (NED): [%.2f, %.2f, %.2f] m/s\n', v_ref_2);
    fprintf('Speed: %.2f m/s\n', norm(v_ref_2));
    fprintf('a_inertial (NED): [%.2f, %.2f, %.2f] m/s^2\n', a_inertial_NED);
    fprintf('a_inertial magnitude: %.2f m/s^2\n\n', norm(a_inertial_NED));
    
    % Construct vectors
    a_gravity_NED = [0; 0; g0];
    ref_vector_1_NED = a_inertial_NED - a_gravity_NED;
    ref_vector_2_NED = [0; 27; 0];
    
    obs_vector_1_raw = [line2.accel_x; line2.accel_y; line2.accel_z];
    obs_vector_1_body = -obs_vector_1_raw;
    obs_vector_2_body = [line2.mag_x; line2.mag_y; line2.mag_z];
    
    fprintf('--- TRIAD Input Vectors ---\n');
    fprintf('Ref Sensed Accel (NED):   [%.2f, %.2f, %.2f] (mag: %.2f)\n', ...
        ref_vector_1_NED, norm(ref_vector_1_NED));
    fprintf('Obs Sensed Accel (Body):  [%.2f, %.2f, %.2f] (mag: %.2f)\n', ...
        obs_vector_1_body, norm(obs_vector_1_body));
    fprintf('Ref Mag (NED):            [%.2f, %.2f, %.2f] (mag: %.2f)\n', ...
        ref_vector_2_NED, norm(ref_vector_2_NED));
    fprintf('Obs Mag (Body):           [%.2f, %.2f, %.2f] (mag: %.2f)\n\n', ...
        obs_vector_2_body, norm(obs_vector_2_body));
    
    % Check vector alignment
    fprintf('--- Vector Alignment Check ---\n');
    accel_dot = dot(obs_vector_1_body / norm(obs_vector_1_body), ...
                    ref_vector_1_NED / norm(ref_vector_1_NED));
    mag_dot = dot(obs_vector_2_body / norm(obs_vector_2_body), ...
                  ref_vector_2_NED / norm(ref_vector_2_NED));
    fprintf('Accel alignment (dot product): %.4f (angle: %.2f deg)\n', ...
        accel_dot, rad2deg(acos(accel_dot)));
    fprintf('Mag alignment (dot product):   %.4f (angle: %.2f deg)\n\n', ...
        mag_dot, rad2deg(acos(mag_dot)));
    
    % TRIAD solution
    B = obs_vector_1_body * ref_vector_1_NED' + obs_vector_2_body * ref_vector_2_NED';
    [U, S, V] = svd(B);
    fprintf('--- SVD Singular Values ---\n');
    fprintf('S1: %.4f, S2: %.4f, S3: %.4f\n\n', S(1,1), S(2,2), S(3,3));
    
    M = diag([1, 1, det(U) * det(V)]);
    R_est_body_to_ned = V * M * U';
    
    % Ground truth
    true_quat = [line2.ref_qw, line2.ref_qx, line2.ref_qy, line2.ref_qz];
    R_true_body_to_ned = quat2rotm(true_quat);
    true_eul = rad2deg(quat2eul(true_quat, 'ZYX'));
    
    fprintf('--- Ground Truth Attitude ---\n');
    fprintf('Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n', ...
        true_eul(3), true_eul(2), true_eul(1));
    fprintf('True Body-to-NED DCM:\n');
    disp(R_true_body_to_ned);
    
    % Estimated attitude
    est_eul = rad2deg(rotm2eul(R_est_body_to_ned, 'ZYX'));
    fprintf('--- Estimated Attitude ---\n');
    fprintf('Roll: %.4f deg, Pitch: %.4f deg, Yaw: %.4f deg\n', ...
        est_eul(3), est_eul(2), est_eul(1));
    fprintf('Estimated Body-to-NED DCM:\n');
    disp(R_est_body_to_ned);
    
    % Error analysis
    est_quat_from_R = rotm2quat(R_est_body_to_ned);
    q_err = quatmultiply(true_quat, quatinv(est_quat_from_R));
    error_angle_rad = 2 * acos(q_err(1));
    if error_angle_rad > pi
        error_angle_rad = 2*pi - error_angle_rad;
    end
    error_angle_deg = rad2deg(error_angle_rad);
    
    fprintf('--- Error Analysis ---\n');
    fprintf('Roll Error:  %.4f deg\n', est_eul(3) - true_eul(3));
    fprintf('Pitch Error: %.4f deg\n', est_eul(2) - true_eul(2));
    fprintf('Yaw Error:   %.4f deg\n', est_eul(1) - true_eul(1));
    fprintf('Total Quaternion Error Angle: %.4f degrees\n\n\n', error_angle_deg);
end
