%% GPS Velocity Low-Pass Filter Analysis
% Test different cutoff frequencies for GPS velocity filtering
% Compare filtered velocity derivatives against reference
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end

%% Configuration
gps_hz = 20;
imu_hz = 1000;
gps_update_interval = imu_hz / gps_hz;

% Test multiple cutoff frequencies
cutoff_frequencies = [0.05, 0.5, 1.0, 2.0, 5.0, 10.0]; % Hz
n_filters = length(cutoff_frequencies);

%% Extract GPS Data at 20 Hz
gps_indices = 1:gps_update_interval:size(data, 1);
n_gps_samples = length(gps_indices) - 1;

time_gps = zeros(n_gps_samples, 1);
gps_Ve_North_raw = zeros(n_gps_samples, 1);
gps_Ve_East_raw = zeros(n_gps_samples, 1);
gps_Ve_Down_raw = zeros(n_gps_samples, 1);

% Reference acceleration (from ref velocity derivative)
ref_accel_North = zeros(n_gps_samples, 1);
ref_accel_East = zeros(n_gps_samples, 1);
ref_accel_Down = zeros(n_gps_samples, 1);

% Filtered velocities (one for each cutoff frequency)
gps_Ve_North_filt = zeros(n_gps_samples, n_filters);
gps_Ve_East_filt = zeros(n_gps_samples, n_filters);
gps_Ve_Down_filt = zeros(n_gps_samples, n_filters);

% Derived accelerations from filtered velocities
gps_accel_North_raw = zeros(n_gps_samples, 1);
gps_accel_East_raw = zeros(n_gps_samples, 1);
gps_accel_Down_raw = zeros(n_gps_samples, 1);

gps_accel_North_filt = zeros(n_gps_samples, n_filters);
gps_accel_East_filt = zeros(n_gps_samples, n_filters);
gps_accel_Down_filt = zeros(n_gps_samples, n_filters);

%% Calculate Alpha for Each Filter
dt_gps = 1 / gps_hz;
alpha = zeros(n_filters, 1);
for f = 1:n_filters
    fc = cutoff_frequencies(f);
    alpha(f) = exp(-2 * pi * fc * dt_gps);
end

fprintf('=== LOW-PASS FILTER PARAMETERS ===\n');
fprintf('GPS Sample Rate: %.1f Hz (dt = %.3f s)\n', gps_hz, dt_gps);
for f = 1:n_filters
    fprintf('Filter %d: fc = %.1f Hz, alpha = %.4f\n', f, cutoff_frequencies(f), alpha(f));
end
fprintf('\n');

%% Process GPS Data
fprintf('Processing GPS data at 20 Hz...\n');

for k = 1:n_gps_samples
    idx_current = gps_indices(k);
    idx_next = gps_indices(k+1);
    
    line_current = data(idx_current, :);
    line_next = data(idx_next, :);
    
    time_gps(k) = line_next.time_ms / 1000;
    
    % Raw GPS velocities
    gps_Ve_North_raw(k) = line_next.gps_Ve_North;
    gps_Ve_East_raw(k) = line_next.gps_Ve_East;
    gps_Ve_Down_raw(k) = line_next.gps_Ve_Down;
    
    % Reference velocities
    ref_Ve_North = line_next.ref_Ve_North;
    ref_Ve_East = line_next.ref_Ve_East;
    ref_Ve_Down = line_next.ref_Ve_Down;
    
    % Apply filters
    if k == 1
        % Initialize filters with first measurement
        for f = 1:n_filters
            gps_Ve_North_filt(k, f) = gps_Ve_North_raw(k);
            gps_Ve_East_filt(k, f) = gps_Ve_East_raw(k);
            gps_Ve_Down_filt(k, f) = gps_Ve_Down_raw(k);
        end
    else
        % Apply exponential moving average filter
        for f = 1:n_filters
            gps_Ve_North_filt(k, f) = alpha(f) * gps_Ve_North_filt(k-1, f) + (1 - alpha(f)) * gps_Ve_North_raw(k);
            gps_Ve_East_filt(k, f) = alpha(f) * gps_Ve_East_filt(k-1, f) + (1 - alpha(f)) * gps_Ve_East_raw(k);
            gps_Ve_Down_filt(k, f) = alpha(f) * gps_Ve_Down_filt(k-1, f) + (1 - alpha(f)) * gps_Ve_Down_raw(k);
        end
        
        % Compute accelerations from raw GPS velocity
        dt = (line_next.time_ms - line_current.time_ms) / 1000;
        gps_accel_North_raw(k) = (gps_Ve_North_raw(k) - gps_Ve_North_raw(k-1)) / dt;
        gps_accel_East_raw(k) = (gps_Ve_East_raw(k) - gps_Ve_East_raw(k-1)) / dt;
        gps_accel_Down_raw(k) = (gps_Ve_Down_raw(k) - gps_Ve_Down_raw(k-1)) / dt;
        
        % Compute accelerations from filtered GPS velocities
        for f = 1:n_filters
            gps_accel_North_filt(k, f) = (gps_Ve_North_filt(k, f) - gps_Ve_North_filt(k-1, f)) / dt;
            gps_accel_East_filt(k, f) = (gps_Ve_East_filt(k, f) - gps_Ve_East_filt(k-1, f)) / dt;
            gps_accel_Down_filt(k, f) = (gps_Ve_Down_filt(k, f) - gps_Ve_Down_filt(k-1, f)) / dt;
        end
        
        % Reference acceleration
        ref_Ve_North_prev = data(idx_current, :).ref_Ve_North;
        ref_Ve_East_prev = data(idx_current, :).ref_Ve_East;
        ref_Ve_Down_prev = data(idx_current, :).ref_Ve_Down;
        
        ref_accel_North(k) = (ref_Ve_North - ref_Ve_North_prev) / dt;
        ref_accel_East(k) = (ref_Ve_East - ref_Ve_East_prev) / dt;
        ref_accel_Down(k) = (ref_Ve_Down - ref_Ve_Down_prev) / dt;
    end
end

fprintf('Processing complete!\n\n');

%% Calculate Errors
error_accel_raw = zeros(n_gps_samples, 1);
error_accel_filt = zeros(n_gps_samples, n_filters);

for k = 2:n_gps_samples
    % Raw error
    diff_North = gps_accel_North_raw(k) - ref_accel_North(k);
    diff_East = gps_accel_East_raw(k) - ref_accel_East(k);
    diff_Down = gps_accel_Down_raw(k) - ref_accel_Down(k);
    error_accel_raw(k) = sqrt(diff_North^2 + diff_East^2 + diff_Down^2);
    
    % Filtered errors
    for f = 1:n_filters
        diff_North = gps_accel_North_filt(k, f) - ref_accel_North(k);
        diff_East = gps_accel_East_filt(k, f) - ref_accel_East(k);
        diff_Down = gps_accel_Down_filt(k, f) - ref_accel_Down(k);
        error_accel_filt(k, f) = sqrt(diff_North^2 + diff_East^2 + diff_Down^2);
    end
end

%% Statistics
fprintf('=== ACCELERATION ESTIMATION ERRORS (RMS) ===\n');
fprintf('Raw GPS differentiation: %.4f m/s²\n', rms(error_accel_raw(2:end)));
for f = 1:n_filters
    fprintf('Filtered (fc = %.1f Hz):     %.4f m/s²\n', cutoff_frequencies(f), rms(error_accel_filt(2:end, f)));
end
fprintf('\n');

fprintf('=== ACCELERATION ESTIMATION ERRORS (Mean Absolute) ===\n');
fprintf('Raw GPS differentiation: %.4f m/s²\n', mean(error_accel_raw(2:end)));
for f = 1:n_filters
    fprintf('Filtered (fc = %.1f Hz):     %.4f m/s²\n', cutoff_frequencies(f), mean(error_accel_filt(2:end, f)));
end
fprintf('\n');

%% Plotting
figure('Position', [50, 50, 1600, 1000]);

% North acceleration
subplot(4, 2, 1);
plot(time_gps, ref_accel_North, 'k-', 'LineWidth', 2); hold on;
plot(time_gps, gps_accel_North_raw, 'r-', 'LineWidth', 1, 'DisplayName', 'Raw GPS');
colors = lines(n_filters);
for f = 1:n_filters
    plot(time_gps, gps_accel_North_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1, ...
        'DisplayName', sprintf('fc=%.1fHz', cutoff_frequencies(f)));
end
ylabel('Accel North (m/s²)');
legend('Reference', 'Raw GPS', sprintf('fc=%.1fHz', cutoff_frequencies(1)), ...
    sprintf('fc=%.1fHz', cutoff_frequencies(2)), sprintf('fc=%.1fHz', cutoff_frequencies(3)), ...
    sprintf('fc=%.1fHz', cutoff_frequencies(4)), sprintf('fc=%.1fHz', cutoff_frequencies(5)), 'Location', 'best');
grid on;
title('Acceleration Estimates from GPS Velocity Differentiation');

% East acceleration
subplot(4, 2, 3);
plot(time_gps, ref_accel_East, 'k-', 'LineWidth', 2); hold on;
plot(time_gps, gps_accel_East_raw, 'r-', 'LineWidth', 1);
for f = 1:n_filters
    plot(time_gps, gps_accel_East_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Accel East (m/s²)');
grid on;

% Down acceleration
subplot(4, 2, 5);
plot(time_gps, ref_accel_Down, 'k-', 'LineWidth', 2); hold on;
plot(time_gps, gps_accel_Down_raw, 'r-', 'LineWidth', 1);
for f = 1:n_filters
    plot(time_gps, gps_accel_Down_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Accel Down (m/s²)');
grid on;

% Total error
subplot(4, 2, 7);
plot(time_gps, error_accel_raw, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Raw GPS'); hold on;
for f = 1:n_filters
    plot(time_gps, error_accel_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1, ...
        'DisplayName', sprintf('fc=%.1fHz', cutoff_frequencies(f)));
end
ylabel('Total Error (m/s²)');
xlabel('Time (s)');
legend('show', 'Location', 'best');
grid on;

% Zoom on North acceleration (0-80s)
subplot(4, 2, 2);
idx_zoom = time_gps <= 80;
plot(time_gps(idx_zoom), ref_accel_North(idx_zoom), 'k-', 'LineWidth', 2); hold on;
plot(time_gps(idx_zoom), gps_accel_North_raw(idx_zoom), 'r-', 'LineWidth', 1);
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_accel_North_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Accel North (m/s²)');
grid on;
title('Zoom: 0-80 seconds');

% Zoom on East acceleration
subplot(4, 2, 4);
plot(time_gps(idx_zoom), ref_accel_East(idx_zoom), 'k-', 'LineWidth', 2); hold on;
plot(time_gps(idx_zoom), gps_accel_East_raw(idx_zoom), 'r-', 'LineWidth', 1);
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_accel_East_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Accel East (m/s²)');
grid on;

% Zoom on Down acceleration
subplot(4, 2, 6);
plot(time_gps(idx_zoom), ref_accel_Down(idx_zoom), 'k-', 'LineWidth', 2); hold on;
plot(time_gps(idx_zoom), gps_accel_Down_raw(idx_zoom), 'r-', 'LineWidth', 1);
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_accel_Down_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Accel Down (m/s²)');
grid on;

% Error comparison bar chart
subplot(4, 2, 8);
rms_errors = [rms(error_accel_raw(2:end)); rms(error_accel_filt(2:end, :))'];
bar_labels = {'Raw'; sprintf('%.1fHz', cutoff_frequencies(1)); sprintf('%.1fHz', cutoff_frequencies(2)); ...
    sprintf('%.1fHz', cutoff_frequencies(3)); sprintf('%.1fHz', cutoff_frequencies(4)); sprintf('%.1fHz', cutoff_frequencies(5))};
bar(rms_errors);
set(gca, 'XTickLabel', bar_labels);
ylabel('RMS Error (m/s²)');
xlabel('Filter Cutoff Frequency');
title('Acceleration Estimation Error Comparison');
grid on;

%% Velocity Comparison
figure('Position', [100, 100, 1600, 900]);

% North velocity
subplot(3, 2, 1);
plot(time_gps, gps_Ve_North_raw, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Raw GPS'); hold on;
for f = 1:n_filters
    plot(time_gps, gps_Ve_North_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1, ...
        'DisplayName', sprintf('fc=%.1fHz', cutoff_frequencies(f)));
end
ylabel('Velocity North (m/s)');
legend('show', 'Location', 'best');
grid on;
title('GPS Velocity: Raw vs Filtered');

% East velocity
subplot(3, 2, 3);
plot(time_gps, gps_Ve_East_raw, 'k-', 'LineWidth', 1.5); hold on;
for f = 1:n_filters
    plot(time_gps, gps_Ve_East_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Velocity East (m/s)');
grid on;

% Down velocity
subplot(3, 2, 5);
plot(time_gps, gps_Ve_Down_raw, 'k-', 'LineWidth', 1.5); hold on;
for f = 1:n_filters
    plot(time_gps, gps_Ve_Down_filt(:, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Velocity Down (m/s)');
xlabel('Time (s)');
grid on;

% Zoom North velocity (0-80s)
subplot(3, 2, 2);
plot(time_gps(idx_zoom), gps_Ve_North_raw(idx_zoom), 'k-', 'LineWidth', 1.5); hold on;
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_Ve_North_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Velocity North (m/s)');
grid on;
title('Zoom: 0-80 seconds');

% Zoom East velocity
subplot(3, 2, 4);
plot(time_gps(idx_zoom), gps_Ve_East_raw(idx_zoom), 'k-', 'LineWidth', 1.5); hold on;
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_Ve_East_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Velocity East (m/s)');
grid on;

% Zoom Down velocity
subplot(3, 2, 6);
plot(time_gps(idx_zoom), gps_Ve_Down_raw(idx_zoom), 'k-', 'LineWidth', 1.5); hold on;
for f = 1:n_filters
    plot(time_gps(idx_zoom), gps_Ve_Down_filt(idx_zoom, f), '-', 'Color', colors(f,:), 'LineWidth', 1);
end
ylabel('Velocity Down (m/s)');
xlabel('Time (s)');
grid on;

fprintf('Analysis complete! Review plots to select optimal cutoff frequency.\n');
