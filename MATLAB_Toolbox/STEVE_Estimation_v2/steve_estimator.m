% Load your data
load("steve_sensor_data.mat");
data = steve_sensor_data;
n = height(data);

% --- Parameters ---
imu_rate_hz = 1000;
gps_update_rate_hz = 40;
loop_step = imu_rate_hz / gps_update_rate_hz;
G = 9.81;
% ** CORRECTED: Gravity in NED is positive on the Z-axis **
g_inertial_ned = [0; 0; G]; 
flight_duration_s = 30;
max_k = flight_duration_s * imu_rate_hz;

% --- Preallocate and Initialize ---
euler_est = zeros(n, 3);
euler_est(:, 2) = 80;
accel_body = zeros(n, 3);
gps_derived_accel_ned = zeros(n, 3);

% --- Liftoff Detection ---
liftoff_detected = false;
liftoff_threshold = 1.5 * G;

% --- Search Parameters ---
search_half_width_deg = 5; 
search_increment_deg = 1.0;

% --- Print Control ---
last_print_time_ms = -inf;
print_interval_ms = 5000;

fprintf('\n--- Starting 3D Attitude Estimation (First 30s) ---\n');
fprintf('Waiting for liftoff (IMU acceleration > %.1f m/s^2)...\n', liftoff_threshold);

% --- Main Loop ---
for k = loop_step:loop_step:max_k
    
    dt = (data.time_ms(k) - data.time_ms(k - loop_step + 1)) / 1000.0;
    
    if k == loop_step
        prev_gps_idx = 1;
    else
        prev_gps_idx = k - loop_step;
    end
    
    % Use the raw IMU reading as accel_body
    accel_body(k,:) = [data.accel_x(k), data.accel_y(k), data.accel_z(k)];
    
    % --- Liftoff Detection ---
    if ~liftoff_detected
        if norm(accel_body(k,:)) > liftoff_threshold
            liftoff_detected = true;
            fprintf('\nLIFTOFF DETECTED at %.2f s!\n', data.time_ms(k)/1000);
            last_print_time_ms = data.time_ms(k);
        else
            euler_est(k, :) = euler_est(prev_gps_idx, :);
            continue;
        end
    end
    
    % Calculate GPS acceleration in NED
    gps_derived_accel_ned(k, 1) = (data.gps_Ve_North(k) - data.gps_Ve_North(prev_gps_idx)) / dt;
    gps_derived_accel_ned(k, 2) = (data.gps_Ve_East(k) - data.gps_Ve_East(prev_gps_idx)) / dt;
    % ** CORRECTED: Positive Down acceleration for NED **
    gps_derived_accel_ned(k, 3) = (data.gps_Ve_Down(k) - data.gps_Ve_Down(prev_gps_idx)) / dt;
    
    % --- 3D Attitude Search ---
    
    last_phi_deg = euler_est(prev_gps_idx, 1);
    last_theta_deg = euler_est(prev_gps_idx, 2);
    last_psi_deg = euler_est(prev_gps_idx, 3);
    
    min_error = inf;
    best_phi_deg = last_phi_deg;
    best_theta_deg = last_theta_deg;
    best_psi_deg = last_psi_deg;
    
    % Define the target vector once per loop
    target_vector = gps_derived_accel_ned(k,:)' - g_inertial_ned;
    
    for phi_deg = last_phi_deg - search_half_width_deg : search_increment_deg : last_phi_deg + search_half_width_deg
        for theta_deg = last_theta_deg - search_half_width_deg : search_increment_deg : last_theta_deg + search_half_width_deg
            for psi_deg = last_psi_deg - search_half_width_deg : search_increment_deg : last_psi_deg + search_half_width_deg
                
                phi = phi_deg * pi/180;
                theta = theta_deg * pi/180;
                psi = psi_deg * pi/180;
                
                R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
                R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
                R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
                R = R_z * R_y * R_x;
                
                % ** CORRECTED: Use the validated physical model for the error calculation **
                error = norm( target_vector - (R * accel_body(k,:)') );
                
                if error < min_error
                    min_error = error;
                    best_phi_deg = phi_deg;
                    best_theta_deg = theta_deg;
                    best_psi_deg = psi_deg;
                end
            end
        end
    end
    
    % Save Best Estimate
    euler_est(k, 1) = best_phi_deg;
    euler_est(k, 2) = best_theta_deg;
    euler_est(k, 3) = best_psi_deg;
    
    % Print Data
    if data.time_ms(k) >= last_print_time_ms + print_interval_ms - 1
        fprintf('\nTime: %.2f s\n', data.time_ms(k)/1000);
        fprintf('  IMU Accel   [X, Y, Z]:  [%.2f, %.2f, %.2f] m/s^2\n', accel_body(k,1), accel_body(k,2), accel_body(k,3));
        fprintf('  GPS Accel   [N, E, D]:  [%.2f, %.2f, %.2f] m/s^2\n', gps_derived_accel_ned(k,1), gps_derived_accel_ned(k,2), gps_derived_accel_ned(k,3));
        fprintf('  Est. Attitude [R, P, Y]:  [%.1f, %.1f, %.1f] deg\n', best_phi_deg, best_theta_deg, best_psi_deg);
        
        last_print_time_ms = data.time_ms(k);
    end
end

fprintf('\n--- Estimation Complete ---\n');
