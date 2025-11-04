% SCRIPT: run_gyro_propagation_v2.m
% A simple demonstration of attitude propagation using gyroscope data,
% with a dedicated warmup phase for bias estimation.

clear; clc; close all;

%% --- 1. Initialization ---
% Initial attitude in Euler angles [roll, pitch, yaw] in degrees
phi   = 0;
theta = 80;
psi   = 0;

% Convert initial Euler angles to a quaternion [w, x, y, z]
q_initial = EulerToQuaternion(phi, theta, psi);

% Define the magnetic field vector (for later use)
m_ref = [27.5550; -2.4169; -16.0849];

% Load sensor data
import_sensor_data; % Your script to load the CSV
sensor_data = phoenix_sensor_sim_output;

num_samples = length(sensor_data.Time_s);
fprintf('Data loaded with %d samples.\n', num_samples);


%% --- 2. Warmup Phase: Gyroscope Bias Estimation ---
warm_up_time = 10; % seconds

% Find all data points within the warmup period
warmup_indices = find(sensor_data.Time_s <= warm_up_time);
if isempty(warmup_indices)
    error('No data found in the specified warmup time.');
end

% Extract gyro data during this period
gyro_warmup_data = [sensor_data.Gyro_X_rad_s(warmup_indices), ...
                    sensor_data.Gyro_Y_rad_s(warmup_indices), ...
                    sensor_data.Gyro_Z_rad_s(warmup_indices)];

% Calculate the average to get the final bias estimate
bg = mean(gyro_warmup_data, 1)'; % Transpose to make it a column vector

fprintf('Warmup complete.\n');
fprintf('Estimated Gyro Bias (rad/s): [%.6f, %.6f, %.6f]\n', bg(1), bg(2), bg(3));


%% --- 3. Propagation Loop ---
% Find the starting index for the main flight phase
start_index = warmup_indices(end) + 1;

% Allocate space for results and set the initial value
results_q = zeros(4, num_samples);
results_q(:, 1:start_index-1) = repmat(q_initial, 1, start_index-1); % Attitude is static during warmup
q = q_initial; % Reset working quaternion

fprintf('Starting propagation loop from sample %d...\n', start_index);

% Loop through all remaining flight data points
for k = start_index:num_samples
    
    % Calculate the time step (dt)
    dt = sensor_data.Time_s(k) - sensor_data.Time_s(k-1);
    
    % Get the bias-corrected angular velocity vector
    omega_corr = [sensor_data.Gyro_X_rad_s(k) - bg(1);
                  sensor_data.Gyro_Y_rad_s(k) - bg(2);
                  sensor_data.Gyro_Z_rad_s(k) - bg(3)];
                  
    % Propagate the quaternion using the gyroscope reading
    q = QuaternionIntegration(q, omega_corr, dt);
    
    % Store the new quaternion
    results_q(:, k) = q;
end

fprintf('Propagation complete.\n');
fprintf('Final Quaternion [w, x, y, z]:   [%.4f, %.4f, %.4f, %.4f]\n', q);
[phi_final, theta_final, psi_final] = QuaternionToEuler(q);
fprintf('Final Euler Angles [R, P, Y]:  [%.2f, %.2f, %.2f] deg\n', phi_final, theta_final, psi_final);


%% --- 4. Plotting Results (Corrected Method) ---

% --- Convert Estimated Quaternions to Euler for Comparison Plot ---
estimated_roll  = zeros(1, num_samples);
estimated_pitch = zeros(1, num_samples);
estimated_yaw   = zeros(1, num_samples);
for k = 1:num_samples
    [r, p, y] = QuaternionToEuler(results_q(:, k));
    estimated_roll(k)  = r;
    estimated_pitch(k) = p;
    estimated_yaw(k)   = y;
end

% --- Calculate Attitude Error in Quaternion Domain ---
error_roll_q  = zeros(1, num_samples);
error_pitch_q = zeros(1, num_samples);
error_yaw_q   = zeros(1, num_samples);

for k = 1:num_samples
    % Get the reference quaternion for this timestep
    q_ref = EulerToQuaternion(sensor_data.Ref_Att_Roll_deg(k), ...
                              sensor_data.Ref_Att_Pitch_deg(k), ...
                              sensor_data.Ref_Att_Yaw_deg(k));
                              
    % Get the estimated quaternion
    q_est = results_q(:, k);
    
    % Calculate the error quaternion: q_err = q_ref * inv(q_est)
    q_err = QuaternionMultiply(q_ref, QuaternionInverse(q_est));
    
    % Convert the small error quaternion to Euler angles (which are now error angles)
    [r_err, p_err, y_err] = QuaternionToEuler(q_err);
    error_roll_q(k)  = r_err;
    error_pitch_q(k) = p_err;
    error_yaw_q(k)   = y_err;
end


% --- PLOT 1: Comparison of Estimated vs. Reference ---
figure;
sgtitle('Gyro Propagation: Estimated vs. Reference Attitude');
% (This plotting code is identical to the previous version)
% Plot Roll
subplot(3, 1, 1);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Roll_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, estimated_roll, 'b--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Roll (deg)');
legend('Reference', 'Estimated (Gyro Only)'); title('Roll Angle Comparison');
% Plot Pitch
subplot(3, 1, 2);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Pitch_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, estimated_pitch, 'b--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Pitch (deg)');
legend('Reference', 'Estimated (Gyro Only)'); title('Pitch Angle Comparison');
% Plot Yaw
subplot(3, 1, 3);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Yaw_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, estimated_yaw, 'b--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Yaw (deg)');
legend('Reference', 'Estimated (Gyro Only)'); title('Yaw Angle Comparison');


% --- PLOT 2: Attitude Estimation Errors (Calculated via Quaternions) ---
figure;
sgtitle('Gyro Propagation: Attitude Estimation Error (Quaternion Method)');

% Plot Roll Error
subplot(3, 1, 1);
plot(sensor_data.Time_s, error_roll_q, 'k', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Error (deg)'); title('Roll Angle Error');

% Plot Pitch Error
subplot(3, 1, 2);
plot(sensor_data.Time_s, error_pitch_q, 'k', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Error (deg)'); title('Pitch Angle Error');

% Plot Yaw Error
subplot(3, 1, 3);
plot(sensor_data.Time_s, error_yaw_q, 'k', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Error (deg)'); title('Yaw Angle Error');

% --- Calculate Maximum Errors ---
max_error_roll = max(abs(error_roll_q));
max_error_pitch = max(abs(error_pitch_q));
max_error_yaw = max(abs(error_yaw_q));

fprintf('Maximum Errors (deg): Roll: %.4f, Pitch: %.4f, Yaw: %.4f\n', ...
        max_error_roll, max_error_pitch, max_error_yaw);

% --- Calculate RMS Errors ---
rms_error_roll = sqrt(mean(error_roll_q.^2));
rms_error_pitch = sqrt(mean(error_pitch_q.^2));
rms_error_yaw = sqrt(mean(error_yaw_q.^2));

fprintf('RMS Errors (deg): Roll: %.4f, Pitch: %.4f, Yaw: %.4f\n', ...
        rms_error_roll, rms_error_pitch, rms_error_yaw);