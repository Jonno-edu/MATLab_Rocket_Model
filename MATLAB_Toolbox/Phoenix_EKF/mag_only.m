% =========================================================================
% Magnetometer-Only Attitude Estimation using TRIAD
% --- FINAL VERSION with Dynamic Motion, Noise, and LOW-PASS FILTER ---
% =========================================================================

% Clear workspace, close figures, and clear command window
clear; close all; clc;
fprintf('--- Starting Advanced TRIAD Simulation with Filtering ---\n\n');


% --- 0. Load Data from MAT-file ---
load('phoenix_ref_pqr_deg.mat');
ref_pqr = phoenix_sensor_sim_output;



% --- 1. Simulation Setup ---
T = ref_pqr.Time_s(end);            % Simulation Time in seconds
dt = ref_pqr.Time_s(2) - ref_pqr.Time_s(1);          % Time step
time = (0:dt:T-dt)';
N = length(time);
fs = 1/dt;          % Sampling frequency


ref_omega = [ref_pqr.Ref_P_deg, ref_pqr.Ref_Q_deg, ref_pqr.Ref_R_deg];
true_omega = deg2rad(ref_omega) + [0.01, 0.01, 0.01];

% --- 3. Generate True Data & Noisy Sensor Readings ---
B_inertial = [25, 5, -15];
mag_noise_std = 0.05;      % Noise standard deviation in uT
fprintf('Generating true attitude and adding sensor noise (std dev = %.2f uT).\n', mag_noise_std);

true_attitude_R = zeros(3, 3, N);
true_attitude_R(:,:,1) = eye(3);
B_body_meas = zeros(N, 3);

for k = 1:N
    if k > 1
        omega_step = true_omega(k-1,:);
        angle_step = norm(omega_step) * dt;
        axis_step = omega_step / (angle_step + eps);
        delta_R = axang2rotm([axis_step, angle_step]);
        true_attitude_R(:,:,k) = delta_R * true_attitude_R(:,:,k-1);
    end
    perfect_mag_reading = (true_attitude_R(:,:,k)' * B_inertial')';
    B_body_meas(k, :) = perfect_mag_reading + randn(1,3) * mag_noise_std;
end

% --- 4. NEW: Apply Low-Pass Filter to Magnetometer Data ---
% This is the critical step to reduce noise before differentiation.
fc = 10.0; % Filter cutoff frequency in Hz. This is a key tuning parameter.
[b, a] = butter(4, fc / (fs/2), 'low');

fprintf('Applying a %.1f Hz low-pass filter to magnetometer data...\n\n', fc);
mag_meas_filtered = filtfilt(b, a, B_body_meas);

% --- 5. Generate the Second Vector (The Derivative) ---
% Now we calculate the derivative from the CLEANED signal.
dBdt_body_meas = diff(mag_meas_filtered) / dt;
dBdt_body_meas = [dBdt_body_meas(1,:); dBdt_body_meas];

% --- 6. Implement TRIAD Algorithm ---
fprintf('Running TRIAD algorithm on filtered data...\n');
triad_attitude_R = zeros(3, 3, N); % This will be Inertial-to-Body

for k = 1:N
    % Use the FILTERED magnetometer data as the primary vector
    v1_body = mag_meas_filtered(k, :);
    v2_body = dBdt_body_meas(k, :);
    
    if norm(cross(v1_body, v2_body)) < 1e-6
        if k > 1, triad_attitude_R(:,:,k) = triad_attitude_R(:,:,k-1); else, triad_attitude_R(:,:,k) = eye(3); end
        continue;
    end
    
    t1_body = v1_body / norm(v1_body);
    t2_body = cross(v1_body, v2_body) / norm(cross(v1_body, v2_body));
    t3_body = cross(t1_body, t2_body);
    R_body = [t1_body', t2_body', t3_body'];
    
    v1_ref = B_inertial;
    v2_ref = -cross(true_omega(k,:), B_inertial);
    
    if norm(cross(v1_ref, v2_ref)) < 1e-6
        if k > 1, triad_attitude_R(:,:,k) = triad_attitude_R(:,:,k-1); else, triad_attitude_R(:,:,k) = eye(3); end
        continue;
    end
    
    t1_ref = v1_ref / norm(v1_ref);
    t2_ref = cross(v1_ref, v2_ref) / norm(cross(v1_ref, v2_ref));
    t3_ref = cross(t1_ref, t2_ref);
    R_ref = [t1_ref', t2_ref', t3_ref'];
    
    triad_attitude_R(:,:,k) = R_body * R_ref';
end
fprintf('Simulation and TRIAD estimation complete.\n\n');

% --- 7. Analyze and Compare Results ---
true_eul = rad2deg(rotm2eul(true_attitude_R, 'ZYX'));
triad_body_to_inertial_R = pagectranspose(triad_attitude_R);
triad_eul = rad2deg(rotm2eul(triad_body_to_inertial_R, 'ZYX'));

true_q = rotm2quat(true_attitude_R);
triad_q = rotm2quat(triad_body_to_inertial_R);
q_error = quatmultiply(quatconj(triad_q), true_q);
scalar_part = abs(q_error(:,1));
angle_error_deg = rad2deg(2 * acos(scalar_part));

% --- 8. Plotting ---
figure('Name', 'TRIAD Attitude Estimation (Filtered)');
subplot(3,1,1); hold on; plot(time, true_eul(:,1), 'k-', 'LineWidth', 2); plot(time, triad_eul(:,1), 'r--'); title('Yaw (Z-axis)'); ylabel('Degrees'); grid on; legend('True', 'TRIAD (Filtered)'); hold off;
subplot(3,1,2); hold on; plot(time, true_eul(:,2), 'k-', 'LineWidth', 2); plot(time, triad_eul(:,2), 'g--'); title('Pitch (Y-axis)'); ylabel('Degrees'); grid on; legend('True', 'TRIAD (Filtered)'); hold off;
subplot(3,1,3); hold on; plot(time, true_eul(:,3), 'k-', 'LineWidth', 2); plot(time, triad_eul(:,3), 'b--'); title('Roll (X-axis)'); ylabel('Degrees'); xlabel('Time (s)'); grid on; legend('True', 'TRIAD (Filtered)'); hold off;


figure('Name', 'Total Quaternion Angle Error (Filtered)');
plot(time, angle_error_deg, 'c', 'LineWidth', 1.5); % Changed color to cyan
title('Total Attitude Error (from Quaternions)');
xlabel('Time (s)'); ylabel('Total Angle Error (degrees)'); grid on;
