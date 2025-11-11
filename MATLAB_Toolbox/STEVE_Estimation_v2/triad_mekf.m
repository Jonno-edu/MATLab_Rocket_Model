%% MEKF Attitude Estimation with Adaptive R by Flight Phase
% Multiplicative Extended Kalman Filter with phase-dependent measurement noise
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end
g0 = 9.80665;

%% MEKF Initialization
x = zeros(6, 1);
P = eye(6);
P(1:3, 1:3) = deg2rad(10)^2 * eye(3);
P(4:6, 4:6) = deg2rad(0.5)^2 * eye(3);

% Process noise
sigma_gyro_noise = deg2rad(0.1);
sigma_bias_walk = deg2rad(0.001);

% Phase detection thresholds
warm_up_threshold = 1.2;
thrust_threshold = 15.0;
coast_threshold = 2.0;
launch_duration = 3.0;
min_thrust_duration = 0.5;

% Adaptive measurement noise by phase
sigma_triad_warmup = deg2rad(5.0);
sigma_triad_launch = deg2rad(50.0);
sigma_triad_thrust = deg2rad(1.0);
sigma_triad_coast = deg2rad(10.0);

fprintf('=== ADAPTIVE R BY FLIGHT PHASE ===\n');
fprintf('Warm-up σ: %.1f° (R = %.4f rad²)\n', rad2deg(sigma_triad_warmup), sigma_triad_warmup^2);
fprintf('Launch   σ: %.1f° (R = %.4f rad²)\n', rad2deg(sigma_triad_launch), sigma_triad_launch^2);
fprintf('Boost    σ: %.1f° (R = %.4f rad²)\n', rad2deg(sigma_triad_thrust), sigma_triad_thrust^2);
fprintf('Coast    σ: %.1f° (R = %.4f rad²)\n\n', rad2deg(sigma_triad_coast), sigma_triad_coast^2);

% Initialize quaternion
q_est = [1; 0; 0; 0];
bias_est = zeros(3, 1);

% GPS update parameters
gps_hz = 20;
imu_hz = 1000;
gps_update_interval = imu_hz / gps_hz;
last_gps_index = 1;

% GPS lock during high acceleration
enable_gps_lock_high_accel = true;
gps_lock_accel_threshold = 4 * g0;
gps_locked = true;
gps_was_locked = true;
gps_reactivation_delay = 2.0;
gps_reactivation_time = -inf;
gps_reactivation_active = false;

% GPS velocity low-pass filter parameters
fc_gps_warmup = 0.1;
fc_gps_other = 0.5;
dt_gps = 1 / gps_hz;

fprintf('GPS Velocity LPF: Adaptive - Warm-up: %.1f Hz, Launch/Boost/Coast: %.1f Hz\n\n', fc_gps_warmup, fc_gps_other);

if enable_gps_lock_high_accel
    fprintf('GPS Lock Enabled: GPS measurements locked above %.1f g (%.1f m/s²)\n', gps_lock_accel_threshold/g0, gps_lock_accel_threshold);
    fprintf('GPS Reactivation Delay: %.1f seconds with increased R\n\n', gps_reactivation_delay);
else
    fprintf('GPS Lock Disabled\n\n');
end

% GPS filter state
gps_Ve_North_filt = 0;
gps_Ve_East_filt = 0;
gps_Ve_Down_filt = 0;
gps_filter_initialized = false;

% State machine variables
current_phase = 0;
thrust_entry_sample = -1;
launch_entry_sample = -1;
min_samples_for_transition = round(min_thrust_duration * imu_hz);
phase_transitions = [];

%% Pre-allocate Storage
n_samples = size(data, 1) - 1;
time_s = zeros(n_samples, 1);
true_roll = zeros(n_samples, 1);
true_pitch = zeros(n_samples, 1);
true_yaw = zeros(n_samples, 1);
mekf_roll = zeros(n_samples, 1);
mekf_pitch = zeros(n_samples, 1);
mekf_yaw = zeros(n_samples, 1);
triad_roll = zeros(n_samples, 1);
triad_pitch = zeros(n_samples, 1);
triad_yaw = zeros(n_samples, 1);
error_angle_mekf = zeros(n_samples, 1);
error_angle_triad = zeros(n_samples, 1);
error_roll_quat = zeros(n_samples, 1);
error_pitch_quat = zeros(n_samples, 1);
error_yaw_quat = zeros(n_samples, 1);
bias_x = zeros(n_samples, 1);
bias_y = zeros(n_samples, 1);
bias_z = zeros(n_samples, 1);
accel_mag_body = zeros(n_samples, 1);
accel_mag_inertial = zeros(n_samples, 1);
flight_phase = zeros(n_samples, 1);
sigma_used = zeros(n_samples, 1);
gps_update_flags = false(n_samples, 1);
gps_lock_flags = false(n_samples, 1);
gps_recovery_flags = false(n_samples, 1);

%% Helper Functions
quatmult = @(q1, q2) [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
                      q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
                      q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
                      q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];
quatconj = @(q) [q(1); -q(2); -q(3); -q(4)];
delta_theta_to_quat = @(dtheta) [1; 0.5*dtheta(1); 0.5*dtheta(2); 0.5*dtheta(3)];
skew = @(w) [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

%% Main Loop
fprintf('Processing %d samples with Adaptive MEKF...\n', n_samples);

for k = 2:n_samples+1
    index = k;
    
    line1 = data(index-1, :);
    line2 = data(index, :);
    dt = (line2.time_ms - line1.time_ms) / 1000;
    if dt == 0, dt = 0.001; end
    
    time_s(k-1) = line2.time_ms / 1000;
    
    % Gyro measurements
    omega_meas = [line2.gyro_x; line2.gyro_y; line2.gyro_z];
    
    % Body accelerometer
    obs_vector_1_body = -[line2.accel_x; line2.accel_y; line2.accel_z];
    accel_mag_body(k-1) = norm(obs_vector_1_body);
    
    %% STATE MACHINE: Flight Phase Detection
    accel_deviation = abs(accel_mag_body(k-1) - g0);
    
    switch current_phase
        case 0  % WARM-UP
            if accel_mag_body(k-1) > (g0 + thrust_threshold)
                current_phase = 3;
                launch_entry_sample = k-1;
                phase_transitions = [phase_transitions; k-1, 0, 3, time_s(k-1)];
                fprintf('Phase transition at t=%.3fs: Warm-up → Launch\n', time_s(k-1));
            end
            
        case 3  % LAUNCH
            time_in_launch = (k-1 - launch_entry_sample) / imu_hz;
            if time_in_launch >= launch_duration
                current_phase = 1;
                thrust_entry_sample = k-1;
                phase_transitions = [phase_transitions; k-1, 3, 1, time_s(k-1)];
                fprintf('Phase transition at t=%.3fs: Launch → Boost\n', time_s(k-1));
            end
            
        case 1  % BOOST
            samples_in_thrust = (k-1) - thrust_entry_sample;
            if samples_in_thrust >= min_samples_for_transition
                if accel_mag_body(k-1) < (g0 - coast_threshold)
                    current_phase = 2;
                    phase_transitions = [phase_transitions; k-1, 1, 2, time_s(k-1)];
                    fprintf('Phase transition at t=%.3fs: Boost → Coast\n', time_s(k-1));
                end
            end
            
        case 2  % COAST
            % Stay in coast
    end
    
    flight_phase(k-1) = current_phase;
    
    %% GPS LOCK LOGIC
    if enable_gps_lock_high_accel
        if accel_mag_body(k-1) > gps_lock_accel_threshold
            if ~gps_locked
                gps_locked = true;
                fprintf('GPS locked at t=%.3fs due to high acceleration (%.1f g)\n', time_s(k-1), accel_mag_body(k-1)/g0);
            end
        elseif accel_mag_body(k-1) < gps_lock_accel_threshold
            if gps_locked
                gps_locked = false;
                fprintf('GPS unlocked at t=%.3fs (%.1f g), allowing measurements to settle\n', time_s(k-1), accel_mag_body(k-1)/g0);
            end
        end
    end
    
    % Detect GPS reactivation
    gps_reactivation = gps_was_locked && ~gps_locked;
    if gps_reactivation
        gps_reactivation_time = time_s(k-1);
        gps_reactivation_active = true;
        fprintf('GPS reactivated at t=%.3fs, increasing R for %.1fs\n', time_s(k-1), gps_reactivation_delay);
    end
    gps_was_locked = gps_locked;
    
    % Check if reactivation period has expired
    if gps_reactivation_active && (time_s(k-1) - gps_reactivation_time >= gps_reactivation_delay)
        gps_reactivation_active = false;
        fprintf('GPS reactivation settling complete at t=%.3fs, R back to normal\n', time_s(k-1));
    end
    
    %% PROPAGATION
    omega_corrected = omega_meas - bias_est;
    
    % Propagate quaternion
    omega_norm = norm(omega_corrected);
    if omega_norm > 1e-8
        q_est = quatmult(q_est, delta_theta_to_quat(omega_corrected * dt));
        q_est = q_est / norm(q_est);
    end
    
    % State transition
    F = zeros(6, 6);
    F(1:3, 1:3) = -skew(omega_corrected);
    F(1:3, 4:6) = -eye(3);
    F_discrete = eye(6) + F * dt;
    
    % Process noise
    Q = zeros(6, 6);
    Q(1:3, 1:3) = sigma_gyro_noise^2 * eye(3) * dt^2;
    Q(4:6, 4:6) = sigma_bias_walk^2 * eye(3) * dt;
    
    % Propagate covariance
    P = F_discrete * P * F_discrete' + Q;
    
    %% MEASUREMENT UPDATE
    gps_update = mod(k-2, gps_update_interval) == 0;
    gps_update_flags(k-1) = gps_update;
    gps_lock_flags(k-1) = gps_locked;
    gps_recovery_flags(k-1) = gps_reactivation_active;
    
    % Skip GPS update if locked
    if gps_locked
        gps_update = false;
    end
    
    if gps_update
        % Set adaptive GPS filter cutoff
        if current_phase == 0
            fc_gps = fc_gps_warmup;
        else
            fc_gps = fc_gps_other;
        end
        alpha_gps = exp(-2 * pi * fc_gps * dt_gps);
        
        % Get and filter GPS velocities
        gps_Ve_North_raw = line2.gps_Ve_North;
        gps_Ve_East_raw = line2.gps_Ve_East;
        gps_Ve_Down_raw = line2.gps_Ve_Down;
        
        if ~gps_filter_initialized
            gps_Ve_North_filt = gps_Ve_North_raw;
            gps_Ve_East_filt = gps_Ve_East_raw;
            gps_Ve_Down_filt = gps_Ve_Down_raw;
            gps_filter_initialized = true;
            gps_Ve_North_filt_prev = gps_Ve_North_filt;
            gps_Ve_East_filt_prev = gps_Ve_East_filt;
            gps_Ve_Down_filt_prev = gps_Ve_Down_filt;
        else
            gps_Ve_North_filt_prev = gps_Ve_North_filt;
            gps_Ve_East_filt_prev = gps_Ve_East_filt;
            gps_Ve_Down_filt_prev = gps_Ve_Down_filt;
            
            gps_Ve_North_filt = alpha_gps * gps_Ve_North_filt + (1 - alpha_gps) * gps_Ve_North_raw;
            gps_Ve_East_filt = alpha_gps * gps_Ve_East_filt + (1 - alpha_gps) * gps_Ve_East_raw;
            gps_Ve_Down_filt = alpha_gps * gps_Ve_Down_filt + (1 - alpha_gps) * gps_Ve_Down_raw;
        end
        
        % Calculate acceleration from filtered GPS
        gps_line1 = data(last_gps_index, :);
        gps_line2 = data(index, :);
        dt_gps_actual = (gps_line2.time_ms - gps_line1.time_ms) / 1000;
        
        if dt_gps_actual > 0
            a_inertial_NED = ([gps_Ve_North_filt; gps_Ve_East_filt; gps_Ve_Down_filt] - ...
                              [gps_Ve_North_filt_prev; gps_Ve_East_filt_prev; gps_Ve_Down_filt_prev]) / dt_gps;
            
            accel_mag_inertial(k-1) = norm(a_inertial_NED);
            
            % Construct TRIAD vectors
            a_gravity_NED = [0; 0; g0];
            ref_vector_1_NED = a_inertial_NED - a_gravity_NED;
            ref_vector_2_NED = [0; 27; 0];
            obs_vector_2_body = [line2.mag_x; line2.mag_y; line2.mag_z];
            
            % Adaptive measurement noise
            switch current_phase
                case 0, sigma_triad = sigma_triad_warmup;
                case 3, sigma_triad = sigma_triad_launch;
                case 1, sigma_triad = sigma_triad_thrust;
                case 2, sigma_triad = sigma_triad_coast;
            end
            
            % Temporarily increase R during GPS reactivation
            if gps_reactivation_active
                sigma_triad = sigma_triad * 20.0;
            end
            
            sigma_used(k-1) = rad2deg(sigma_triad);
            R_meas = sigma_triad^2 * eye(3);
            
            % TRIAD solution
            B = obs_vector_1_body * ref_vector_1_NED' + obs_vector_2_body * ref_vector_2_NED';
            [U, ~, V] = svd(B);
            M = diag([1, 1, det(U) * det(V)]);
            R_triad = V * M * U';
            q_triad = rotm2quat(R_triad)';
            if q_triad(1) < 0, q_triad = -q_triad; end
            
            % Compute error
            q_err = quatmult(quatconj(q_est), q_triad);
            if q_err(1) < 0, q_err = -q_err; end
            z = 2 * q_err(2:4);
            
            % Kalman update
            H = [eye(3), zeros(3, 3)];
            S = H * P * H' + R_meas;
            K = P * H' / S;
            x = K * z;
            
            % Apply correction
            delta_q = delta_theta_to_quat(x(1:3));
            q_est = quatmult(q_est, delta_q);
            q_est = q_est / norm(q_est);
            bias_est = bias_est + x(4:6);
            x(1:3) = zeros(3, 1);
            
            % Update covariance
            P = (eye(6) - K * H) * P;
            last_gps_index = index;
            
            % Store TRIAD
            triad_eul = rad2deg(rotm2eul(R_triad, 'ZYX'));
            triad_roll(k-1) = triad_eul(3);
            triad_pitch(k-1) = triad_eul(2);
            triad_yaw(k-1) = triad_eul(1);
        end
    else
        % Propagate values
        if k > 2
            accel_mag_inertial(k-1) = accel_mag_inertial(k-2);
            sigma_used(k-1) = sigma_used(k-2);
            triad_roll(k-1) = triad_roll(k-2);
            triad_pitch(k-1) = triad_pitch(k-2);
            triad_yaw(k-1) = triad_yaw(k-2);
        end
    end
    
    %% Store Results
    true_quat = [line2.ref_qw, line2.ref_qx, line2.ref_qy, line2.ref_qz];
    true_eul = rad2deg(quat2eul(true_quat, 'ZYX'));
    true_roll(k-1) = true_eul(3);
    true_pitch(k-1) = true_eul(2);
    true_yaw(k-1) = true_eul(1);
    
    mekf_eul = rad2deg(quat2eul([q_est(1), q_est(2), q_est(3), q_est(4)], 'ZYX'));
    mekf_roll(k-1) = mekf_eul(3);
    mekf_pitch(k-1) = mekf_eul(2);
    mekf_yaw(k-1) = mekf_eul(1);
    
    % QUATERNION-BASED ERROR CALCULATION
    q_err_mekf = quatmultiply(true_quat, quatinv([q_est(1), q_est(2), q_est(3), q_est(4)]));
    error_angle_rad = 2 * acos(min(abs(q_err_mekf(1)), 1.0));
    error_angle_mekf(k-1) = rad2deg(error_angle_rad);
    
    % Convert quaternion error to Euler angle errors
    error_eul = rad2deg(quat2eul(q_err_mekf, 'ZYX'));
    error_roll_quat(k-1) = error_eul(3);
    error_pitch_quat(k-1) = error_eul(2);
    error_yaw_quat(k-1) = error_eul(1);
    
    if gps_update && exist('R_triad', 'var')
        q_err_triad = quatmultiply(true_quat, quatinv(rotm2quat(R_triad)));
        error_angle_rad = 2 * acos(min(abs(q_err_triad(1)), 1.0));
        error_angle_triad(k-1) = rad2deg(error_angle_rad);
    elseif k > 2
        error_angle_triad(k-1) = error_angle_triad(k-2);
    end
    
    bias_x(k-1) = rad2deg(bias_est(1));
    bias_y(k-1) = rad2deg(bias_est(2));
    bias_z(k-1) = rad2deg(bias_est(3));
    
    if mod(k, 10000) == 0
        fprintf('Processed %d/%d (%.1f%%) - Phase: %d\n', k-1, n_samples, 100*(k-1)/n_samples, current_phase);
    end
end

fprintf('Processing complete!\n\n');

%% Phase Statistics
phase_names = {'Warm-up', 'Boost', 'Coast', 'Launch'};
phase_colors = {'g', 'r', 'b', 'm'};

fprintf('=== FLIGHT PHASE DISTRIBUTION ===\n');
for p = 0:3
    phase_samples = sum(flight_phase == p);
    fprintf('%s: %d samples (%.2f s, %.1f%%)\n', phase_names{p+1}, ...
        phase_samples, phase_samples/imu_hz, 100*phase_samples/n_samples);
end

if ~isempty(phase_transitions)
    fprintf('\n=== PHASE TRANSITIONS ===\n');
    for i = 1:size(phase_transitions, 1)
        fprintf('%.3f s: %s → %s\n', phase_transitions(i,4), ...
            phase_names{phase_transitions(i,2)+1}, phase_names{phase_transitions(i,3)+1});
    end
end
fprintf('\n');

%% MEKF Error Statistics by Phase (Quaternion-Based)
fprintf('=== MEKF ERROR STATISTICS BY PHASE (Quaternion-Based) ===\n');
for p = 0:3
    phase_mask = flight_phase == p;
    if any(phase_mask)
        fprintf('\n%s (%d samples):\n', phase_names{p+1}, sum(phase_mask));
        fprintf('  Roll  - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(error_roll_quat(phase_mask))), ...
            std(error_roll_quat(phase_mask)), ...
            max(abs(error_roll_quat(phase_mask))));
        fprintf('  Pitch - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(error_pitch_quat(phase_mask))), ...
            std(error_pitch_quat(phase_mask)), ...
            max(abs(error_pitch_quat(phase_mask))));
        fprintf('  Yaw   - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(error_yaw_quat(phase_mask))), ...
            std(error_yaw_quat(phase_mask)), ...
            max(abs(error_yaw_quat(phase_mask))));
        fprintf('  Total - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(error_angle_mekf(phase_mask)), std(error_angle_mekf(phase_mask)), ...
            max(error_angle_mekf(phase_mask)));
    end
end

fprintf('\n=== OVERALL MEKF STATISTICS (Quaternion-Based) ===\n');
fprintf('Roll  - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
    mean(abs(error_roll_quat)), std(error_roll_quat), max(abs(error_roll_quat)));
fprintf('Pitch - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
    mean(abs(error_pitch_quat)), std(error_pitch_quat), max(abs(error_pitch_quat)));
fprintf('Yaw   - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
    mean(abs(error_yaw_quat)), std(error_yaw_quat), max(abs(error_yaw_quat)));
fprintf('Total - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
    mean(error_angle_mekf), std(error_angle_mekf), max(error_angle_mekf));

fprintf('\n=== FINAL GYRO BIAS ESTIMATES ===\n');
fprintf('X: %.4f deg/s\n', bias_x(end));
fprintf('Y: %.4f deg/s\n', bias_y(end));
fprintf('Z: %.4f deg/s\n', bias_z(end));

%% Comprehensive Tabbed Plotting
fig = figure('Name', 'MEKF Performance Analysis', 'Position', [50, 50, 1600, 900]);
tabgroup = uitabgroup(fig);

% Color scheme
color_true = [0, 0, 0];
color_mekf = [0, 0.4470, 0.7410];
color_triad = [0.8500, 0.3250, 0.0980];
color_error = [0.6350, 0.0780, 0.1840];

%% TAB 1: Attitude Estimates
tab1 = uitab(tabgroup, 'Title', 'Attitude Estimates');
axes('Parent', tab1);

gps_idx = find(gps_update_flags);

subplot(3,1,1);
hold on; grid on;
plot(time_s, true_roll, 'Color', color_true, 'LineWidth', 1.5, 'DisplayName', 'True');
plot(time_s, mekf_roll, 'Color', color_mekf, 'LineWidth', 1.2, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_roll(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
ylabel('Roll (deg)');
title('Attitude Estimates vs True Values');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,2);
hold on; grid on;
plot(time_s, true_pitch, 'Color', color_true, 'LineWidth', 1.5, 'DisplayName', 'True');
plot(time_s, mekf_pitch, 'Color', color_mekf, 'LineWidth', 1.2, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_pitch(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
ylabel('Pitch (deg)');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,3);
hold on; grid on;
plot(time_s, true_yaw, 'Color', color_true, 'LineWidth', 1.5, 'DisplayName', 'True');
plot(time_s, mekf_yaw, 'Color', color_mekf, 'LineWidth', 1.2, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_yaw(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
xlabel('Time (s)');
ylabel('Yaw (deg)');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

%% TAB 2: Quaternion-Based Errors
tab2 = uitab(tabgroup, 'Title', 'Attitude Errors');
axes('Parent', tab2);

% Calculate TRIAD errors at GPS update points
triad_error_roll = zeros(size(error_roll_quat));
triad_error_pitch = zeros(size(error_pitch_quat));
triad_error_yaw = zeros(size(error_yaw_quat));

for i = 1:length(gps_idx)
    idx = gps_idx(i);
    triad_error_roll(idx) = triad_roll(idx) - true_roll(idx);
    triad_error_pitch(idx) = triad_pitch(idx) - true_pitch(idx);
    triad_error_yaw(idx) = triad_yaw(idx) - true_yaw(idx);
end

subplot(4,1,1);
hold on; grid on;
plot(time_s, error_roll_quat, 'Color', color_mekf, 'LineWidth', 1, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_error_roll(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
ylabel('Roll Error (deg)');
title('Quaternion-Based Attitude Errors');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(4,1,2);
hold on; grid on;
plot(time_s, error_pitch_quat, 'Color', color_mekf, 'LineWidth', 1, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_error_pitch(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
ylabel('Pitch Error (deg)');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(4,1,3);
hold on; grid on;
plot(time_s, error_yaw_quat, 'Color', color_mekf, 'LineWidth', 1, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), triad_error_yaw(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
ylabel('Yaw Error (deg)');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(4,1,4);
hold on; grid on;
plot(time_s, error_angle_mekf, 'Color', color_mekf, 'LineWidth', 1.2, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), error_angle_triad(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 4, 'DisplayName', 'TRIAD');
end
xlabel('Time (s)');
ylabel('Total Error (deg)');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

%% TAB 3: Gyro Bias Estimates
tab3 = uitab(tabgroup, 'Title', 'Gyro Bias');
axes('Parent', tab3);

subplot(3,1,1);
hold on; grid on;
plot(time_s, bias_x, 'LineWidth', 1.2);
ylabel('X Bias (deg/s)');
title('Gyro Bias Estimates');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,2);
hold on; grid on;
plot(time_s, bias_y, 'LineWidth', 1.2);
ylabel('Y Bias (deg/s)');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,3);
hold on; grid on;
plot(time_s, bias_z, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Z Bias (deg/s)');
add_phase_background(time_s, flight_phase, phase_colors);

%% TAB 4: Acceleration Profiles
tab4 = uitab(tabgroup, 'Title', 'Acceleration');
axes('Parent', tab4);

subplot(3,1,1);
hold on; grid on;
plot(time_s, accel_mag_body / g0, 'LineWidth', 1.2, 'Color', [0.2, 0.6, 0.8]);
yline(1, 'k--', 'LineWidth', 1, 'DisplayName', '1g');
if enable_gps_lock_high_accel
    yline(gps_lock_accel_threshold / g0, 'r--', 'LineWidth', 1.5, ...
        'DisplayName', 'GPS Lock Threshold');
end
ylabel('Body Accel (g)');
title('Acceleration Profiles');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,2);
hold on; grid on;
plot(time_s, accel_mag_inertial, 'LineWidth', 1.2, 'Color', [0.8, 0.4, 0.2]);
ylabel('Inertial Accel (m/s²)');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,3);
hold on; grid on;
plot(time_s, flight_phase, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Flight Phase');
yticks(0:3);
yticklabels(phase_names);
ylim([-0.5, 3.5]);
grid on;

%% TAB 5: Adaptive Parameters
tab5 = uitab(tabgroup, 'Title', 'Adaptive R & GPS');
axes('Parent', tab5);

subplot(3,1,1);
hold on; grid on;
plot(time_s, sigma_used, 'LineWidth', 1.5, 'Color', [0.4940, 0.1840, 0.5560]);
ylabel('σ_{triad} (deg)');
title('Adaptive Measurement Noise');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,2);
hold on; grid on;
gps_update_times = time_s(gps_update_flags);
if ~isempty(gps_update_times)
    stem(gps_update_times, ones(size(gps_update_times)), 'Marker', 'none', ...
        'LineWidth', 1, 'Color', [0, 0.7, 0]);
end
ylabel('GPS Updates');
ylim([0, 1.5]);
yticks([0, 1]);
yticklabels({'No', 'Yes'});
add_phase_background(time_s, flight_phase, phase_colors);

subplot(3,1,3);
hold on; grid on;
locked_times = time_s(gps_lock_flags);
if ~isempty(locked_times)
    stem(locked_times, ones(size(locked_times)), 'Marker', 'none', ...
        'LineWidth', 1, 'Color', [0.8, 0, 0]);
end
recovery_times = time_s(gps_recovery_flags);
if ~isempty(recovery_times)
    stem(recovery_times, 0.5 * ones(size(recovery_times)), 'Marker', 'o', ...
        'LineWidth', 1.5, 'Color', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
end
xlabel('Time (s)');
ylabel('GPS Lock');
ylim([0, 1.5]);
yticks([0, 1]);
yticklabels({'Unlocked', 'Locked'});
add_phase_background(time_s, flight_phase, phase_colors);

%% TAB 6: Error Statistics
tab6 = uitab(tabgroup, 'Title', 'Error Statistics');
axes('Parent', tab6);

% Prepare data for boxplots - handle variable lengths properly
phase_labels = {};
roll_data = [];
roll_groups = [];
pitch_data = [];
pitch_groups = [];
yaw_data = [];
yaw_groups = [];
total_data = [];
total_groups = [];

for p = 0:3
    phase_mask = flight_phase == p;
    if any(phase_mask)
        phase_labels{end+1} = phase_names{p+1};
        
        % Roll errors
        roll_vals = abs(error_roll_quat(phase_mask));
        roll_data = [roll_data; roll_vals];
        roll_groups = [roll_groups; p * ones(length(roll_vals), 1)];
        
        % Pitch errors
        pitch_vals = abs(error_pitch_quat(phase_mask));
        pitch_data = [pitch_data; pitch_vals];
        pitch_groups = [pitch_groups; p * ones(length(pitch_vals), 1)];
        
        % Yaw errors
        yaw_vals = abs(error_yaw_quat(phase_mask));
        yaw_data = [yaw_data; yaw_vals];
        yaw_groups = [yaw_groups; p * ones(length(yaw_vals), 1)];
        
        % Total errors
        total_vals = error_angle_mekf(phase_mask);
        total_data = [total_data; total_vals];
        total_groups = [total_groups; p * ones(length(total_vals), 1)];
    end
end

subplot(2,2,1);
boxplot(roll_data, roll_groups, 'Labels', phase_labels);
ylabel('Roll Error (deg)');
title('Roll Error by Phase');
grid on;

subplot(2,2,2);
boxplot(pitch_data, pitch_groups, 'Labels', phase_labels);
ylabel('Pitch Error (deg)');
title('Pitch Error by Phase');
grid on;

subplot(2,2,3);
boxplot(yaw_data, yaw_groups, 'Labels', phase_labels);
ylabel('Yaw Error (deg)');
title('Yaw Error by Phase');
grid on;

subplot(2,2,4);
boxplot(total_data, total_groups, 'Labels', phase_labels);
ylabel('Total Error (deg)');
title('Total Error by Phase');
grid on;

%% TAB 7: MEKF vs TRIAD
tab7 = uitab(tabgroup, 'Title', 'MEKF vs TRIAD');
axes('Parent', tab7);

subplot(2,1,1);
hold on; grid on;
plot(time_s, error_angle_mekf, 'Color', color_mekf, 'LineWidth', 1.2, 'DisplayName', 'MEKF');
if ~isempty(gps_idx)
    plot(time_s(gps_idx), error_angle_triad(gps_idx), '.', 'Color', color_triad, ...
        'MarkerSize', 6, 'DisplayName', 'TRIAD');
end
ylabel('Error (deg)');
title('MEKF vs TRIAD Comparison');
legend('Location', 'best');
add_phase_background(time_s, flight_phase, phase_colors);

subplot(2,1,2);
hold on; grid on;
if ~isempty(gps_idx)
    error_diff = error_angle_mekf(gps_idx) - error_angle_triad(gps_idx);
    plot(time_s(gps_idx), error_diff, 'o', 'MarkerSize', 3, 'Color', [0.5, 0, 0.5]);
    yline(0, 'k--', 'LineWidth', 1);
end
xlabel('Time (s)');
ylabel('MEKF - TRIAD (deg)');
title('Error Difference (negative = MEKF better)');
add_phase_background(time_s, flight_phase, phase_colors);

%% TAB 8: Phase Transitions Detail
if ~isempty(phase_transitions)
    tab8 = uitab(tabgroup, 'Title', 'Transition Detail');
    axes('Parent', tab8);
    
    n_trans = min(size(phase_transitions, 1), 4);
    for i = 1:n_trans
        t_trans = phase_transitions(i, 4);
        zoom_window = 2.0;
        zoom_mask = (time_s >= t_trans - zoom_window) & (time_s <= t_trans + zoom_window);
        
        subplot(n_trans, 1, i);
        hold on; grid on;
        plot(time_s(zoom_mask), error_angle_mekf(zoom_mask), 'LineWidth', 1.5);
        xline(t_trans, 'r--', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Error (deg)');
        title(sprintf('%s → %s (t=%.2fs)', ...
            phase_names{phase_transitions(i,2)+1}, ...
            phase_names{phase_transitions(i,3)+1}, t_trans));
    end
end

fprintf('\nAll plots generated successfully in tabbed figure!\n');

%% Helper Function
function add_phase_background(time_s, flight_phase, phase_colors)
    y_limits = ylim;
    hold on;
    
    phase_changes = [1; find(diff(flight_phase) ~= 0) + 1; length(flight_phase)];
    
    for i = 1:length(phase_changes)-1
        idx_start = phase_changes(i);
        idx_end = phase_changes(i+1);
        phase = flight_phase(idx_start);
        
        if phase >= 0 && phase <= 3
            color = phase_colors{phase + 1};
            patch([time_s(idx_start), time_s(idx_end), time_s(idx_end), time_s(idx_start)], ...
                  [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], ...
                  color, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
    
    uistack(findobj(gca, 'Type', 'patch'), 'bottom');
    ylim(y_limits);
end
