% Test script for non-spinning frame TVC control
clear; clc;

%% Test Configuration
% Current rocket orientation (Body frame relative to Earth)
roll_current = 270;      % degrees
pitch_current = 80;     % degrees  
yaw_current = 0;        % degrees

% Desired orientation
roll_desired = 0;       % Roll is not controlled
pitch_desired = 90;      % degrees
yaw_desired = 0;        % degrees

% Controller gain (simple P controller)
K = 1.0;

%% Step 1: Create quaternions from Euler angles
q_BE = euler_to_quat(roll_current, pitch_current, yaw_current);
q_ref = euler_to_quat(roll_desired, pitch_desired, yaw_desired);

fprintf('=== Current Rocket State ===\n');
fprintf('Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n\n', ...
    roll_current, pitch_current, yaw_current);

%% Step 2: Extract Euler angles from q_BE
[phi, theta, psi] = quat_to_euler(q_BE);

fprintf('=== Extracted from Quaternion ===\n');
fprintf('Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n\n', ...
    rad2deg(phi), rad2deg(theta), rad2deg(psi));

%% Step 3: Create non-spinning frame quaternion (zero roll)
q_BE_nonspinning = euler_to_quat(0, rad2deg(theta), rad2deg(psi));

fprintf('=== Non-Spinning Frame ===\n');
[phi_ns, theta_ns, psi_ns] = quat_to_euler(q_BE_nonspinning);
fprintf('Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n\n', ...
    rad2deg(phi_ns), rad2deg(theta_ns), rad2deg(psi_ns));

%% Step 4: Compute attitude errors in non-spinning frame
q_error = quat_multiply(q_ref, quat_conjugate(q_BE_nonspinning));
[~, e_theta, e_psi] = quat_to_euler(q_error);

fprintf('=== Attitude Errors (Non-Spinning Frame) ===\n');
fprintf('Pitch error: %.2f deg\n', rad2deg(e_theta));
fprintf('Yaw error: %.2f deg\n\n', rad2deg(e_psi));

%% Step 5: Controller (simple P control in non-spinning frame)
delta_Y_ns = K * e_theta;  % Nozzle deflection around Y-axis
delta_Z_ns = K * e_psi;     % Nozzle deflection around Z-axis

fprintf('=== Controller Output (Non-Spinning Frame) ===\n');
fprintf('Delta_Y: %.4f rad (%.2f deg)\n', delta_Y_ns, rad2deg(delta_Y_ns));
fprintf('Delta_Z: %.4f rad (%.2f deg)\n\n', delta_Z_ns, rad2deg(delta_Z_ns));

%% Step 6: Rotate to body frame using current roll angle
R_roll = [cos(phi), -sin(phi);
          sin(phi),  cos(phi)];

delta_body = R_roll * [delta_Y_ns; delta_Z_ns];
delta_Y_body = delta_body(1);
delta_Z_body = delta_body(2);

fprintf('=== Final Actuator Commands (Body Frame) ===\n');
fprintf('Delta_Y_body: %.4f rad (%.2f deg)\n', delta_Y_body, rad2deg(delta_Y_body));
fprintf('Delta_Z_body: %.4f rad (%.2f deg)\n\n', delta_Z_body, rad2deg(delta_Z_body));

%% Verification: Show rotation effect
fprintf('=== Rotation Effect ===\n');
fprintf('Roll angle used for transformation: %.2f deg\n', rad2deg(phi));
fprintf('Magnitude (non-spinning): %.4f rad\n', norm([delta_Y_ns, delta_Z_ns]));
fprintf('Magnitude (body frame): %.4f rad\n', norm([delta_Y_body, delta_Z_body]));
fprintf('Magnitudes match: %s\n\n', string(abs(norm([delta_Y_ns, delta_Z_ns]) - norm([delta_Y_body, delta_Z_body])) < 1e-10));

%% Helper Functions

function q = euler_to_quat(roll_deg, pitch_deg, yaw_deg)
    % Convert Euler angles (degrees) to quaternion
    roll = deg2rad(roll_deg);
    pitch = deg2rad(pitch_deg);
    yaw = deg2rad(yaw_deg);
    
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    
    q = zeros(4,1);
    q(1) = cr * cp * cy + sr * sp * sy;  % w
    q(2) = sr * cp * cy - cr * sp * sy;  % x
    q(3) = cr * sp * cy + sr * cp * sy;  % y
    q(4) = cr * cp * sy - sr * sp * cy;  % z
end

function [roll, pitch, yaw] = quat_to_euler(q)
    % Convert quaternion to Euler angles (radians)
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi/2;
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
end

function q_conj = quat_conjugate(q)
    % Compute quaternion conjugate
    q_conj = [q(1); -q(2); -q(3); -q(4)];
end

function q_result = quat_multiply(q1, q2)
    % Multiply two quaternions
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    
    q_result = zeros(4,1);
    q_result(1) = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q_result(2) = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q_result(3) = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q_result(4) = w1*z2 + x1*y2 - y1*x2 + z1*w2;
end
