% run_MEKF_mag_aided.m
%
% Magnetometer-aided MEKF for spinning rocket attitude estimation
% Uses gyro for propagation and magnetometer for updates
% Low bias process noise assumes good warmup calibration
% Initial orientation: [0, 80, 0] deg (roll, pitch, yaw)
% Error calculated via quaternion method (more robust)

clear;
clc;
close all;

% --- 1. Load Sensor Data ---
import_sensor_data;
sensor_data = phoenix_sensor_sim_output;

% --- 2. Define Constants and Tuning Parameters ---
% Geomagnetic Field Vector (NED frame, for your location)
mag_ref_NED = [27.5550; -2.4169; -16.0849]; % microTesla
mag_ref_norm = mag_ref_NED / norm(mag_ref_NED); % Normalize

% Sampling
dt_gyro = 0.001; % Assume 1000 Hz gyro (adjust based on your data)
dt_mag = 0.01;   % 100 Hz magnetometer updates (adjust if different)

% Number of samples
N = height(sensor_data);

% Warmup period (adjust to match your data)
warmup_time = 30; % seconds
warmup_idx = find(sensor_data.Time_s > warmup_time, 1);

% --- 3. Process and Measurement Noise Tuning ---
% Gyro noise parameters (adjust based on your IMU datasheet)
gyro_noise_std = 0.001; % rad/s (angle random walk)
Q_gyro = (gyro_noise_std^2 * dt_gyro) * eye(3); % Attitude process noise

% Bias process noise - VERY LOW for 60s flight after warmup
bias_drift_std = 1e-5; % rad/s^2 (very slow drift)
Q_bias = (bias_drift_std^2 * dt_gyro) * eye(3);

% Combined process noise
Q = blkdiag(Q_gyro, Q_bias);

% Magnetometer measurement noise
mag_noise_std = 0.5; % microTesla (adjust based on your sensor)
R_mag = (mag_noise_std^2) * eye(3);

% --- 4. Initialize State ---
% State: [attitude_error (3x1); bias (3x1)]
% Global state: quaternion q (4x1), bias b (3x1)

% Initial orientation: [roll, pitch, yaw] = [0, 80, 0] degrees
roll_init = 0 * pi/180;
pitch_init = 80 * pi/180;
yaw_init = 0 * pi/180;

% Convert Euler angles to quaternion (ZYX convention)
q = euler_to_quat(roll_init, pitch_init, yaw_init);

fprintf('Initial quaternion from [0, 80, 0] deg: [%.4f, %.4f, %.4f, %.4f]\n', ...
        q(1), q(2), q(3), q(4));

% Initial bias estimate (from warmup)
bias_init = zeros(3, 1);

% Estimate bias during warmup
if warmup_idx > 10
    gyro_warmup = [sensor_data.Gyro_X_rad_s(1:warmup_idx), ...
                   sensor_data.Gyro_Y_rad_s(1:warmup_idx), ...
                   sensor_data.Gyro_Z_rad_s(1:warmup_idx)];
    bias_init = mean(gyro_warmup, 1)';
    fprintf('Warmup complete. Bias estimate: [%.6f, %.6f, %.6f] rad/s\n', ...
            bias_init(1), bias_init(2), bias_init(3));
end

bias = bias_init;

% Initial covariance
% Smaller initial attitude uncertainty since we know the starting orientation
P = blkdiag((1*pi/180)^2 * eye(3), (0.001)^2 * eye(3)); % [attitude_error; bias]

% --- 5. Storage for Results ---
q_history = zeros(N, 4);
bias_history = zeros(N, 3);
euler_history = zeros(N, 3); % [roll, pitch, yaw] in degrees
euler_error_history = zeros(N, 3); % Error calculated via quaternion method

% --- 6. Main Filter Loop ---
fprintf('Running MEKF with magnetometer updates...\n');

for k = 1:N
    t = sensor_data.Time_s(k);
    
    % Get sensor measurements
    gyro = [sensor_data.Gyro_X_rad_s(k); 
            sensor_data.Gyro_Y_rad_s(k); 
            sensor_data.Gyro_Z_rad_s(k)];
    
    mag_meas = [sensor_data.Mag_X_uT(k);
                sensor_data.Mag_Y_uT(k);
                sensor_data.Mag_Z_uT(k)];
    mag_meas_norm = mag_meas / norm(mag_meas); % Normalize
    
    % Get reference attitude
    roll_ref = sensor_data.Ref_Att_Roll_deg(k) * pi/180;
    pitch_ref = sensor_data.Ref_Att_Pitch_deg(k) * pi/180;
    yaw_ref = sensor_data.Ref_Att_Yaw_deg(k) * pi/180;
    q_ref = euler_to_quat(roll_ref, pitch_ref, yaw_ref);
    
    % --- PREDICTION STEP ---
    % Bias-corrected angular velocity
    omega = gyro - bias;
    
    % Propagate quaternion using bias-corrected gyro
    q = quaternion_propagate(q, omega, dt_gyro);
    
    % State transition matrix (continuous-time)
    Omega_skew = skew_symmetric(omega);
    F = [-Omega_skew, -eye(3);
         zeros(3,3),  zeros(3,3)];
    
    % Discretize using first-order approximation
    Phi = eye(6) + F * dt_gyro;
    
    % Propagate covariance
    P = Phi * P * Phi' + Q;
    
    % --- UPDATE STEP (Magnetometer) ---
    if sensor_data.New_Mag_Data_Flag(k) && k > warmup_idx
        % Rotation matrix from current quaternion estimate
        C_bn = quat_to_dcm(q); % Body-to-NED
        
        % Predicted magnetometer measurement
        mag_pred = C_bn' * mag_ref_norm; % NED-to-Body
        
        % Innovation (measurement residual)
        y = mag_meas_norm - mag_pred;
        
        % Measurement Jacobian H = [dh/d(att_err), dh/d(bias)]
        % For magnetometer: dh/d(att_err) = -[C_bn * mag_ref]_x (skew-symmetric)
        H = [skew_symmetric(C_bn' * mag_ref_norm), zeros(3,3)];
        
        % Innovation covariance
        S = H * P * H' + R_mag;
        
        % Kalman gain
        K = P * H' / S;
        
        % State update
        delta_x = K * y;
        att_error = delta_x(1:3);
        bias_update = delta_x(4:6);
        
        % Update quaternion (multiplicative correction)
        delta_q = axis_angle_to_quat(att_error);
        q = quaternion_multiply(q, delta_q);
        q = q / norm(q); % Normalize
        
        % Update bias (additive correction with low gain due to low Q_bias)
        bias = bias + bias_update;
        
        % Update covariance (Joseph form for numerical stability)
        I_KH = eye(6) - K * H;
        P = I_KH * P * I_KH' + K * R_mag * K';
    end
    
    % --- Calculate Error via Quaternion Method ---
    % q_error = q_ref * q_est^(-1)
    q_error = quaternion_multiply(q_ref, quaternion_conjugate(q));
    
    % Convert error quaternion to Euler angles
    euler_error = quat_to_euler(q_error) * 180/pi; % degrees
    
    % --- Store Results ---
    q_history(k, :) = q';
    bias_history(k, :) = bias';
    euler_history(k, :) = quat_to_euler(q) * 180/pi; % Convert to degrees
    euler_error_history(k, :) = euler_error';
    
    % Progress indicator
    if mod(k, 1000) == 0
        fprintf('Progress: %.1f%%\n', 100*k/N);
    end
end

fprintf('MEKF complete!\n');

% --- 7. Plot Results ---
figure('Name', 'MEKF Attitude Estimation');

subplot(3,1,1);
plot(sensor_data.Time_s, euler_history(:,1), 'b', 'LineWidth', 1.5);
hold on;
plot(sensor_data.Time_s, sensor_data.Ref_Att_Roll_deg, 'r--', 'LineWidth', 1);
ylabel('Roll (deg)');
legend('Estimated', 'Reference', 'Location', 'best');
grid on;
title('MEKF with Magnetometer Aiding - Attitude Estimation');

subplot(3,1,2);
plot(sensor_data.Time_s, euler_history(:,2), 'b', 'LineWidth', 1.5);
hold on;
plot(sensor_data.Time_s, sensor_data.Ref_Att_Pitch_deg, 'r--', 'LineWidth', 1);
ylabel('Pitch (deg)');
legend('Estimated', 'Reference', 'Location', 'best');
grid on;

subplot(3,1,3);
plot(sensor_data.Time_s, euler_history(:,3), 'b', 'LineWidth', 1.5);
hold on;
plot(sensor_data.Time_s, sensor_data.Ref_Att_Yaw_deg, 'r--', 'LineWidth', 1);
ylabel('Yaw (deg)');
xlabel('Time (s)');
legend('Estimated', 'Reference', 'Location', 'best');
grid on;

% Plot Attitude Errors (Quaternion Method)
figure('Name', 'Attitude Estimation Errors');

subplot(3,1,1);
plot(sensor_data.Time_s, euler_error_history(:,1), 'k', 'LineWidth', 1.5);
ylabel('Roll Error (deg)');
grid on;
title('Attitude Estimation Errors (Quaternion Method)');

subplot(3,1,2);
plot(sensor_data.Time_s, euler_error_history(:,2), 'k', 'LineWidth', 1.5);
ylabel('Pitch Error (deg)');
grid on;

subplot(3,1,3);
plot(sensor_data.Time_s, euler_error_history(:,3), 'k', 'LineWidth', 1.5);
ylabel('Yaw Error (deg)');
xlabel('Time (s)');
grid on;

% Plot Total Attitude Error Magnitude
figure('Name', 'Total Attitude Error');
total_error = sqrt(sum(euler_error_history.^2, 2));
plot(sensor_data.Time_s, total_error, 'LineWidth', 1.5);
ylabel('Total Attitude Error (deg)');
xlabel('Time (s)');
grid on;
title('Total Attitude Error Magnitude');

% Plot bias estimates
figure('Name', 'Gyro Bias Estimates');
plot(sensor_data.Time_s, bias_history(:,1:3) * 180/pi, 'LineWidth', 1.5);
ylabel('Bias (deg/s)');
xlabel('Time (s)');
legend('X-axis', 'Y-axis', 'Z-axis', 'Location', 'best');
grid on;
title('Gyro Bias Estimation Over Time');

% Print final statistics
fprintf('\n--- Final Statistics (Post-Warmup) ---\n');
post_warmup = sensor_data.Time_s > warmup_time;

fprintf('\n*** Attitude Errors (Quaternion Method) ***\n');
fprintf('Roll RMS Error: %.4f deg\n', rms(euler_error_history(post_warmup, 1)));
fprintf('Pitch RMS Error: %.4f deg\n', rms(euler_error_history(post_warmup, 2)));
fprintf('Yaw RMS Error: %.4f deg\n', rms(euler_error_history(post_warmup, 3)));
fprintf('Total RMS Error: %.4f deg\n', rms(total_error(post_warmup)));

fprintf('\nFinal Bias: [%.6f, %.6f, %.6f] rad/s\n', ...
        bias(1), bias(2), bias(3));

% ========== HELPER FUNCTIONS ==========

function q = euler_to_quat(roll, pitch, yaw)
    % Convert Euler angles (ZYX convention) to quaternion
    % Input: roll, pitch, yaw in radians
    % Output: quaternion [w; x; y; z]
    
    % ZYX (yaw-pitch-roll) convention
    cr = cos(roll/2);
    sr = sin(roll/2);
    cp = cos(pitch/2);
    sp = sin(pitch/2);
    cy = cos(yaw/2);
    sy = sin(yaw/2);
    
    % Quaternion components (scalar-first format)
    w = cr*cp*cy + sr*sp*sy;
    x = sr*cp*cy - cr*sp*sy;
    y = cr*sp*cy + sr*cp*sy;
    z = cr*cp*sy - sr*sp*cy;
    
    q = [w; x; y; z];
    q = q / norm(q); % Normalize
end

function q_new = quaternion_propagate(q, omega, dt)
    % Propagate quaternion using angular velocity
    omega_norm = norm(omega);
    if omega_norm < 1e-12
        q_new = q;
        return;
    end
    
    % Quaternion derivative
    theta = omega_norm * dt;
    u = omega / omega_norm;
    
    delta_q = [cos(theta/2); u * sin(theta/2)];
    q_new = quaternion_multiply(q, delta_q);
    q_new = q_new / norm(q_new);
end

function q_prod = quaternion_multiply(q1, q2)
    % Hamilton product: q1 * q2
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    
    w = w1*w2 - dot(v1, v2);
    v = w1*v2 + w2*v1 + cross(v1, v2);
    
    q_prod = [w; v];
end

function q_conj = quaternion_conjugate(q)
    % Compute quaternion conjugate (inverse for unit quaternions)
    % q* = [w; -x; -y; -z]
    q_conj = [q(1); -q(2:4)];
end

function C = quat_to_dcm(q)
    % Convert quaternion to direction cosine matrix (body-to-NED)
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    C = [1-2*(y^2+z^2),   2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2),   2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function euler = quat_to_euler(q)
    % Convert quaternion to Euler angles [roll, pitch, yaw] (ZYX sequence)
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
    pitch = asin(2*(w*y - z*x));
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    
    euler = [roll; pitch; yaw];
end

function q = axis_angle_to_quat(aa)
    % Convert axis-angle (small angle) to quaternion
    angle = norm(aa);
    if angle < 1e-12
        q = [1; 0; 0; 0];
        return;
    end
    
    axis = aa / angle;
    q = [cos(angle/2); axis * sin(angle/2)];
end

function S = skew_symmetric(v)
    % Create skew-symmetric matrix from vector
    S = [    0, -v(3),  v(2);
          v(3),     0, -v(1);
         -v(2),  v(1),     0];
end
