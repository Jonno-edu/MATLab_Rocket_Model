%% Rocket Flight Phase Detection and TRIAD Error Analysis
% Detect flight phases from accelerometer with enforced sequential transitions
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end
g0 = 9.80665;

%% Configuration
gps_hz = 20;
imu_hz = 1000;
gps_update_interval = imu_hz / gps_hz;

% Phase detection thresholds
warm_up_threshold = 1.2;      % m/s² deviation from g (stationary if |accel - g| < threshold)
thrust_threshold = 15.0;      % m/s² (thrust phase if accel > g + threshold)
coast_threshold = 2;        % m/s² (coast phase if accel < g - coast_threshold)

% Hysteresis parameters to prevent flickering
min_thrust_duration = 0.5;    % seconds - minimum time in thrust before allowing coast transition
min_samples_for_transition = round(min_thrust_duration * imu_hz);

fprintf('=== FLIGHT PHASE DETECTION (STATE MACHINE) ===\n');
fprintf('Phase sequence: Warm-up → Thrust → Coast (no backward transitions)\n');
fprintf('Warm-up: |accel - g| < %.2f m/s²\n', warm_up_threshold);
fprintf('Thrust:  accel > %.2f m/s² (%.2fg)\n', g0 + thrust_threshold, (g0 + thrust_threshold)/g0);
fprintf('Coast:   accel < %.2f m/s² (%.2fg)\n', g0 - coast_threshold, (g0 - coast_threshold)/g0);
fprintf('Min thrust duration: %.2f s\n\n', min_thrust_duration);

%% GPS velocity low-pass filter
fc_gps = 0.5;
dt_gps = 1 / gps_hz;
alpha_gps = exp(-2 * pi * fc_gps * dt_gps);

%% Pre-allocate Storage
n_samples = size(data, 1) - 1;
time_s = zeros(n_samples, 1);
accel_mag_body = zeros(n_samples, 1);
accel_mag_inertial = zeros(n_samples, 1);
flight_phase = zeros(n_samples, 1); % 0=warm-up, 1=thrust, 2=coast

true_roll = zeros(n_samples, 1);
true_pitch = zeros(n_samples, 1);
true_yaw = zeros(n_samples, 1);
triad_roll = zeros(n_samples, 1);
triad_pitch = zeros(n_samples, 1);
triad_yaw = zeros(n_samples, 1);
error_angle_triad = zeros(n_samples, 1);
gps_update_flags = false(n_samples, 1);

%% Helper Functions
quatmult = @(q1, q2) [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
                      q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
                      q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
                      q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];

%% GPS Filter State
gps_Ve_North_filt = 0;
gps_Ve_East_filt = 0;
gps_Ve_Down_filt = 0;
gps_filter_initialized = false;
last_gps_index = 1;

%% State Machine Variables
current_phase = 0;  % Start in warm-up
thrust_entry_sample = -1;  % Track when we entered thrust phase
phase_transitions = []; % Log transitions for debugging

%% Main Loop - Calculate TRIAD and Detect Phases
fprintf('Processing %d samples...\n', n_samples);

for k = 2:n_samples+1
    index = k;
    
    line1 = data(index-1, :);
    line2 = data(index, :);
    
    time_s(k-1) = line2.time_ms / 1000;
    
    % Body-frame accelerometer magnitude
    obs_vector_1_body = -[line2.accel_x; line2.accel_y; line2.accel_z];
    accel_mag_body(k-1) = norm(obs_vector_1_body);
    
    % Ground truth
    true_quat = [line2.ref_qw, line2.ref_qx, line2.ref_qy, line2.ref_qz];
    true_eul = rad2deg(quat2eul(true_quat, 'ZYX'));
    true_roll(k-1) = true_eul(3);
    true_pitch(k-1) = true_eul(2);
    true_yaw(k-1) = true_eul(1);
    
    % Check if GPS update available
    gps_update = mod(k-2, gps_update_interval) == 0;
    gps_update_flags(k-1) = gps_update;
    
    if gps_update
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
        
        % Calculate inertial acceleration from filtered GPS
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
            
            % TRIAD solution
            B = obs_vector_1_body * ref_vector_1_NED' + obs_vector_2_body * ref_vector_2_NED';
            [U, ~, V] = svd(B);
            M = diag([1, 1, det(U) * det(V)]);
            R_triad = V * M * U';
            q_triad = rotm2quat(R_triad)';
            if q_triad(1) < 0, q_triad = -q_triad; end
            
            % Store TRIAD estimate
            triad_eul = rad2deg(rotm2eul(R_triad, 'ZYX'));
            triad_roll(k-1) = triad_eul(3);
            triad_pitch(k-1) = triad_eul(2);
            triad_yaw(k-1) = triad_eul(1);
            
            % Compute TRIAD error
            q_err_triad = quatmultiply(true_quat, quatinv(rotm2quat(R_triad)));
            error_angle_rad = 2 * acos(abs(q_err_triad(1)));
            error_angle_triad(k-1) = rad2deg(error_angle_rad);
            
            last_gps_index = index;
        end
    else
        % Propagate values
        if k > 2
            accel_mag_inertial(k-1) = accel_mag_inertial(k-2);
            triad_roll(k-1) = triad_roll(k-2);
            triad_pitch(k-1) = triad_pitch(k-2);
            triad_yaw(k-1) = triad_yaw(k-2);
            error_angle_triad(k-1) = error_angle_triad(k-2);
        end
    end
    
    %% STATE MACHINE: Detect flight phase with enforced sequential transitions
    accel_deviation = abs(accel_mag_body(k-1) - g0);
    
    % State machine logic
    switch current_phase
        case 0  % WARM-UP phase
            % Transition to THRUST if acceleration exceeds threshold
            if accel_mag_body(k-1) > (g0 + thrust_threshold)
                current_phase = 1;
                thrust_entry_sample = k-1;
                phase_transitions = [phase_transitions; k-1, 0, 1, time_s(k-1)];
                fprintf('Phase transition at t=%.3fs (sample %d): Warm-up → Thrust\n', time_s(k-1), k-1);
            end
            
        case 1  % THRUST phase
            % Check if we've been in thrust long enough to consider coast transition
            samples_in_thrust = (k-1) - thrust_entry_sample;
            
            if samples_in_thrust >= min_samples_for_transition
                % Transition to COAST if acceleration drops below threshold
                if accel_mag_body(k-1) < (g0 - coast_threshold)
                    current_phase = 2;
                    phase_transitions = [phase_transitions; k-1, 1, 2, time_s(k-1)];
                    fprintf('Phase transition at t=%.3fs (sample %d): Thrust → Coast\n', time_s(k-1), k-1);
                end
            end
            
        case 2  % COAST phase
            % Once in coast, stay in coast (no backward transitions)
            % Nothing to do
    end
    
    % Store current phase
    flight_phase(k-1) = current_phase;
    
    if mod(k, 10000) == 0
        fprintf('Processed %d/%d samples (%.1f%%) - Current phase: %d\n', ...
            k-1, n_samples, 100*(k-1)/n_samples, current_phase);
    end
end

fprintf('Processing complete!\n\n');

%% Phase Statistics
phase_names = {'Warm-up (Pad)', 'Thrust (Launch)', 'Coast (Apogee)'};
phase_colors = {'g', 'r', 'b'};

fprintf('=== FLIGHT PHASE DISTRIBUTION ===\n');
for p = 0:2
    phase_samples = sum(flight_phase == p);
    phase_time = phase_samples / imu_hz;
    fprintf('%s: %d samples (%.2f s, %.1f%%)\n', phase_names{p+1}, ...
        phase_samples, phase_time, 100*phase_samples/n_samples);
end

if ~isempty(phase_transitions)
    fprintf('\n=== PHASE TRANSITIONS ===\n');
    for i = 1:size(phase_transitions, 1)
        fprintf('%.3f s: %s → %s\n', phase_transitions(i,4), ...
            phase_names{phase_transitions(i,2)+1}, phase_names{phase_transitions(i,3)+1});
    end
end
fprintf('\n');

%% TRIAD Error Statistics by Phase
fprintf('=== TRIAD ERROR STATISTICS BY PHASE (GPS updates only) ===\n');
gps_indices = find(gps_update_flags);

for p = 0:2
    phase_gps_indices = gps_indices(flight_phase(gps_indices) == p);
    
    if ~isempty(phase_gps_indices)
        fprintf('\n%s (%d GPS updates):\n', phase_names{p+1}, length(phase_gps_indices));
        fprintf('  Roll  - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(triad_roll(phase_gps_indices) - true_roll(phase_gps_indices))), ...
            std(triad_roll(phase_gps_indices) - true_roll(phase_gps_indices)), ...
            max(abs(triad_roll(phase_gps_indices) - true_roll(phase_gps_indices))));
        fprintf('  Pitch - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(triad_pitch(phase_gps_indices) - true_pitch(phase_gps_indices))), ...
            std(triad_pitch(phase_gps_indices) - true_pitch(phase_gps_indices)), ...
            max(abs(triad_pitch(phase_gps_indices) - true_pitch(phase_gps_indices))));
        fprintf('  Yaw   - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(abs(triad_yaw(phase_gps_indices) - true_yaw(phase_gps_indices))), ...
            std(triad_yaw(phase_gps_indices) - true_yaw(phase_gps_indices)), ...
            max(abs(triad_yaw(phase_gps_indices) - true_yaw(phase_gps_indices))));
        fprintf('  Total - Mean: %.4f°, Std: %.4f°, Max: %.4f°\n', ...
            mean(error_angle_triad(phase_gps_indices)), ...
            std(error_angle_triad(phase_gps_indices)), ...
            max(error_angle_triad(phase_gps_indices)));
        fprintf('  Accel - Body: %.2f m/s², Inertial: %.2f m/s²\n', ...
            mean(accel_mag_body(phase_gps_indices)), ...
            mean(accel_mag_inertial(phase_gps_indices)));
    else
        fprintf('\n%s: No GPS updates in this phase\n', phase_names{p+1});
    end
end

%% Plotting
figure('Position', [50, 50, 1600, 1100]);

% Flight phase timeline with transitions marked
subplot(6,2,[1,2]);
hold on;
for p = 0:2
    phase_mask = flight_phase == p;
    if any(phase_mask)
        area(time_s, phase_mask * (p+1), 'FaceColor', phase_colors{p+1}, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
end
% Mark transitions
if ~isempty(phase_transitions)
    for i = 1:size(phase_transitions, 1)
        xline(phase_transitions(i,4), 'k--', 'LineWidth', 2);
    end
end
ylabel('Flight Phase');
ylim([0, 3.5]);
yticks([1, 2, 3]);
yticklabels(phase_names);
grid on;
title('Flight Phase Detection (State Machine - Sequential Transitions Only)');
xlabel('Time (s)');

% Accelerometer magnitude (body frame) with phase overlays
subplot(6,2,3);
plot(time_s, accel_mag_body, 'k-', 'LineWidth', 1); hold on;
yline(g0, 'g--', 'g', 'LineWidth', 1.5);
yline(g0 + thrust_threshold, 'r--', 'Thrust Threshold', 'LineWidth', 1);
yline(g0 - coast_threshold, 'b--', 'Coast Threshold', 'LineWidth', 1);
if ~isempty(phase_transitions)
    for i = 1:size(phase_transitions, 1)
        xline(phase_transitions(i,4), 'k--', 'LineWidth', 1.5);
    end
end
ylabel('Body Accel (m/s²)');
grid on;
legend('Measured', 'Location', 'best');

% Inertial acceleration magnitude (from GPS)
subplot(6,2,4);
plot(time_s, accel_mag_inertial, 'k-', 'LineWidth', 1); hold on;
if ~isempty(phase_transitions)
    for i = 1:size(phase_transitions, 1)
        xline(phase_transitions(i,4), 'k--', 'LineWidth', 1.5);
    end
end
ylabel('Inertial Accel (m/s²)');
grid on;
title('From GPS Differentiation');

% TRIAD error over time
subplot(6,2,[5,6]);
plot(time_s(gps_update_flags), error_angle_triad(gps_update_flags), 'k.', 'MarkerSize', 3); hold on;
for p = 0:2
    phase_gps_mask = gps_update_flags & (flight_phase == p);
    if any(phase_gps_mask)
        plot(time_s(phase_gps_mask), error_angle_triad(phase_gps_mask), '.', ...
            'Color', phase_colors{p+1}, 'MarkerSize', 5);
    end
end
if ~isempty(phase_transitions)
    for i = 1:size(phase_transitions, 1)
        xline(phase_transitions(i,4), 'k--', 'LineWidth', 1.5);
    end
end
ylabel('TRIAD Error (deg)');
xlabel('Time (s)');
legend('All', phase_names{1}, phase_names{2}, phase_names{3}, 'Location', 'best');
grid on;
title('TRIAD Attitude Error by Flight Phase');

% Error by phase - box plot
subplot(6,2,7);
error_by_phase = cell(3,1);
for p = 0:2
    phase_gps_indices = gps_indices(flight_phase(gps_indices) == p);
    if ~isempty(phase_gps_indices)
        error_by_phase{p+1} = error_angle_triad(phase_gps_indices);
    else
        error_by_phase{p+1} = [];
    end
end
boxplot([error_by_phase{1}; error_by_phase{2}; error_by_phase{3}], ...
        [zeros(length(error_by_phase{1}),1); ones(length(error_by_phase{2}),1); 2*ones(length(error_by_phase{3}),1)], ...
        'Labels', phase_names);
ylabel('TRIAD Error (deg)');
grid on;
title('Error Distribution by Phase');

% Mean error comparison
subplot(6,2,8);
mean_errors = zeros(3,1);
for p = 0:2
    phase_gps_indices = gps_indices(flight_phase(gps_indices) == p);
    if ~isempty(phase_gps_indices)
        mean_errors(p+1) = mean(error_angle_triad(phase_gps_indices));
    end
end
bar(1:3, mean_errors, 'FaceColor', 'flat', 'CData', [0 1 0; 1 0 0; 0 0 1]);
set(gca, 'XTickLabel', phase_names);
ylabel('Mean TRIAD Error (deg)');
grid on;
title('Mean Error by Phase');

% Roll, Pitch, Yaw errors by phase
subplot(6,2,9);
hold on;
for p = 0:2
    phase_gps_indices = gps_indices(flight_phase(gps_indices) == p);
    if ~isempty(phase_gps_indices)
        roll_err = mean(abs(triad_roll(phase_gps_indices) - true_roll(phase_gps_indices)));
        pitch_err = mean(abs(triad_pitch(phase_gps_indices) - true_pitch(phase_gps_indices)));
        yaw_err = mean(abs(triad_yaw(phase_gps_indices) - true_yaw(phase_gps_indices)));
        bar(p+1, [roll_err, pitch_err, yaw_err], 'grouped');
    end
end
set(gca, 'XTickLabel', phase_names);
ylabel('Mean Error (deg)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on;
title('Axis Errors by Phase');

% Sample count by phase
subplot(6,2,10);
phase_counts = [sum(flight_phase==0), sum(flight_phase==1), sum(flight_phase==2)];
bar(1:3, phase_counts, 'FaceColor', 'flat', 'CData', [0 1 0; 1 0 0; 0 0 1]);
set(gca, 'XTickLabel', phase_names);
ylabel('Sample Count');
grid on;
title('Samples per Phase');

% Phase acceleration characteristics
subplot(6,2,11);
hold on;
for p = 0:2
    phase_mask = flight_phase == p;
    if any(phase_mask)
        histogram(accel_mag_body(phase_mask), 50, 'FaceColor', phase_colors{p+1}, 'FaceAlpha', 0.5);
    end
end
xlabel('Accelerometer Magnitude (m/s²)');
ylabel('Count');
legend(phase_names, 'Location', 'best');
grid on;
title('Acceleration Distribution by Phase');

% TRIAD error histogram by phase
subplot(6,2,12);
hold on;
for p = 0:2
    phase_gps_indices = gps_indices(flight_phase(gps_indices) == p);
    if ~isempty(phase_gps_indices)
        histogram(error_angle_triad(phase_gps_indices), 30, 'FaceColor', phase_colors{p+1}, 'FaceAlpha', 0.5);
    end
end
xlabel('TRIAD Error (deg)');
ylabel('Count');
legend(phase_names, 'Location', 'best');
grid on;
title('Error Distribution by Phase');

fprintf('\nAnalysis complete!\n');
