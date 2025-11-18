%% Multi-Phase Adaptive PVA Kalman Filter for GPS Data
% Position-Velocity-Acceleration filter with process noise (Q) adapted
% to specific flight phases (transients, steady-burn, coasting).
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end
g0 = 9.80665;
gps_hz = 20;
dt = 1 / gps_hz;

fprintf('=== MULTI-PHASE ADAPTIVE PVA KALMAN FILTER ===\n');
fprintf('GPS Rate: %d Hz (dt = %.4f s)\n\n', gps_hz, dt);

%% Extract GPS measurements at 20 Hz
n_samples = size(data, 1);
imu_hz = 1000;
gps_interval = imu_hz / gps_hz;

% Downsample to GPS rate
gps_indices = 1:gps_interval:n_samples;
n_gps = length(gps_indices);

time_gps   = zeros(n_gps, 1);
gps_pos_N  = zeros(n_gps, 1);
gps_pos_E  = zeros(n_gps, 1);
gps_pos_D  = zeros(n_gps, 1);
gps_vel_N  = zeros(n_gps, 1);
gps_vel_E  = zeros(n_gps, 1);
gps_vel_D  = zeros(n_gps, 1);

% Reference data for comparison
ref_pos_N  = zeros(n_gps, 1);
ref_pos_E  = zeros(n_gps, 1);
ref_pos_D  = zeros(n_gps, 1);
ref_vel_N  = zeros(n_gps, 1);
ref_vel_E  = zeros(n_gps, 1);
ref_vel_D  = zeros(n_gps, 1);

for i = 1:n_gps
    idx  = gps_indices(i);
    line = data(idx, :);
    
    time_gps(i) = line.time_ms / 1000;
    
    gps_pos_N(i) = line.gps_Xe_North;
    gps_pos_E(i) = line.gps_Xe_East;
    gps_pos_D(i) = line.gps_Xe_Down;
    
    gps_vel_N(i) = line.gps_Ve_North;
    gps_vel_E(i) = line.gps_Ve_East;
    gps_vel_D(i) = line.gps_Ve_Down;
    
    ref_pos_N(i) = line.ref_Xe_North;
    ref_pos_E(i) = line.ref_Xe_East;
    ref_pos_D(i) = line.ref_Xe_Down;
    
    ref_vel_N(i) = line.ref_Ve_North;
    ref_vel_E(i) = line.ref_Ve_East;
    ref_vel_D(i) = line.ref_Ve_Down;
end

%% Multi-Phase Adaptive Filter Initialization
% Define event times and transient windows
t_launch    = 40.0; % s
t_cutout    = 100.0;% s
t_transient = 3.0;  % s, window before/after events

% Define adaptive process noise sigmas for each phase (tunable)
sigma_warmup    = 0.8;   % Quiet phase before launch
sigma_transient = 10.0;  % High dynamics at launch/cutout
sigma_flight    = 5.0;   % Stable burn phase
sigma_coast     = 10.0;  % Post-burn coasting phase



fprintf('Multi-Phase Adaptive Q settings:\n');
fprintf(' - Warm-up (t < %.1fs):             σ = %.2f\n', t_launch - t_transient, sigma_warmup);
fprintf(' - Launch Transient (%.1fs-%.1fs):  σ = %.2f\n', t_launch - t_transient, t_launch + t_transient, sigma_transient);
fprintf(' - Mid-Burn (%.1fs-%.1fs):        σ = %.2f\n', t_launch + t_transient, t_cutout - t_transient, sigma_flight);
fprintf(' - Cutout Transient (%.1fs-%.1fs):  σ = %.2f\n', t_cutout - t_transient, t_cutout + t_transient, sigma_transient);
fprintf(' - Coasting (t > %.1fs):            σ = %.2f\n\n', t_cutout + t_transient, sigma_coast);

% State: [x, y, z, vx, vy, vz, ax, ay, az]'
x       = zeros(9, 1);
x(1:3)  = [gps_pos_N(1); gps_pos_E(1); gps_pos_D(1)];
x(4:6)  = [gps_vel_N(1); gps_vel_E(1); gps_vel_D(1)];

% Initial covariance
P               = eye(9);
P(1:3, 1:3)     = 10^2 * eye(3);
P(4:6, 4:6)     = 1^2  * eye(3);
P(7:9, 7:9)     = 5^2  * eye(3);

% State transition matrix
F = [eye(3), dt*eye(3), 0.5*dt^2*eye(3);
     zeros(3), eye(3), dt*eye(3);
     zeros(3), zeros(3), eye(3)];

% Process noise shaping matrix
G = [0.5*dt^2*eye(3);
     dt*eye(3);
     eye(3)];

% Measurement noise
sigma_pos = 5.0;   % meters
sigma_vel = 0.5;   % m/s
R         = blkdiag(sigma_pos^2 * eye(3), sigma_vel^2 * eye(3));
H         = [eye(6), zeros(6, 3)];

%% Storage arrays
pos_est     = zeros(n_gps, 3);
vel_est     = zeros(n_gps, 3);
accel_est   = zeros(n_gps, 3);
active_sigma = zeros(n_gps, 1);

% Store accel covariance diag(P_aa) over time
P_accel = zeros(n_gps, 3);  % [Var(ax), Var(ay), Var(az)]

pos_est(1, :)   = x(1:3)';
vel_est(1, :)   = x(4:6)';
accel_est(1, :) = x(7:9)';
P_accel(1, :)   = diag(P(7:9, 7:9))';

%% Run Adaptive PVA Filter
fprintf('Running multi-phase adaptive filter...\n');
for k = 1:n_gps
    % --- Adaptive Process Noise Selection ---
    t_current = time_gps(k);
    if t_current < (t_launch - t_transient)
        current_sigma = sigma_warmup;
    elseif t_current >= (t_launch - t_transient) && t_current < (t_launch + t_transient)
        current_sigma = sigma_transient;
    elseif t_current >= (t_launch + t_transient) && t_current < (t_cutout - t_transient)
        current_sigma = sigma_flight;
    elseif t_current >= (t_cutout - t_transient) && t_current <= (t_cutout + t_transient)
        current_sigma = sigma_transient;
    else
        current_sigma = sigma_coast;
    end
    
    % Update Process Noise Matrix Q
    Q = G * (current_sigma^2 * eye(3)) * G' * dt;
    active_sigma(k) = current_sigma;
    
    if k > 1
        % --- Standard Kalman Filter Steps ---
        % Prediction step
        x = F * x;
        P = F * P * F' + Q;
        
        % Measurement
        z = [gps_pos_N(k); gps_pos_E(k); gps_pos_D(k);
             gps_vel_N(k); gps_vel_E(k); gps_vel_D(k)];
        
        % Innovation
        y = z - H * x;
        S = H * P * H' + R;
        K = P * H' / S;
        
        % Update
        x = x + K * y;
        P = (eye(9) - K * H) * P;
    end
    
    % Store results
    pos_est(k, :)   = x(1:3)';
    vel_est(k, :)   = x(4:6)';
    accel_est(k, :) = x(7:9)';
    P_accel(k, :)   = diag(P(7:9, 7:9))';  % accel variances over time
end
fprintf('Processing complete!\n\n');

%% Low-pass filter GPS velocity (simple 1st-order IIR)
vel_gps      = [gps_vel_N, gps_vel_E, gps_vel_D];
vel_gps_filt = zeros(size(vel_gps));

% Time constant and alpha for exponential smoothing
tau_vel  = 0.5;                  % s, tweak as needed
alpha_vel = dt / (tau_vel + dt); % first-order low-pass coefficient

vel_gps_filt(1, :) = vel_gps(1, :);
for k = 2:n_gps
    vel_gps_filt(k, :) = vel_gps_filt(k-1, :) + alpha_vel * (vel_gps(k, :) - vel_gps_filt(k-1, :));
end

%% Compute raw, filtered-derivative and reference acceleration
accel_raw = zeros(n_gps, 3);
accel_lpf = zeros(n_gps, 3);
ref_accel = zeros(n_gps, 3);

for k = 2:n_gps
    dt_actual = time_gps(k) - time_gps(k-1);
    if dt_actual > 0
        % 3D velocity differences (unfiltered GPS)
        dv_gps = [gps_vel_N(k) - gps_vel_N(k-1);
                  gps_vel_E(k) - gps_vel_E(k-1);
                  gps_vel_D(k) - gps_vel_D(k-1)];
        % 3D velocity differences (low-pass filtered GPS)
        dv_gps_filt = vel_gps_filt(k, :)' - vel_gps_filt(k-1, :)';
        % 3D velocity differences (reference)
        dv_ref = [ref_vel_N(k) - ref_vel_N(k-1);
                  ref_vel_E(k) - ref_vel_E(k-1);
                  ref_vel_D(k) - ref_vel_D(k-1)];
        
        accel_raw(k, :) = (dv_gps      / dt_actual).';
        accel_lpf(k, :) = (dv_gps_filt / dt_actual).';
        ref_accel(k, :) = (dv_ref      / dt_actual).';
    end
end

%% Compute magnitudes and errors
accel_est_mag = sqrt(sum(accel_est.^2, 2));
accel_raw_mag = sqrt(sum(accel_raw.^2, 2));
accel_lpf_mag = sqrt(sum(accel_lpf.^2, 2));
ref_accel_mag = sqrt(sum(ref_accel.^2, 2));

error_kf  = accel_est_mag - ref_accel_mag;
error_raw = accel_raw_mag - ref_accel_mag;
error_lpf = accel_lpf_mag - ref_accel_mag;

% Per-axis KF acceleration error for consistency plots
accel_error = accel_est - ref_accel;     % [e_ax, e_ay, e_az]
sigma_accel = sqrt(P_accel);            % 1-sigma bounds from P for ax, ay, az

%% Statistics
fprintf('=== ERROR STATISTICS ===\n');
fprintf('KF  Error - Mean: %.4f m/s², RMS: %.4f m/s²\n', ...
    mean(abs(error_kf)), rms(error_kf));
fprintf('Raw Error - Mean: %.4f m/s², RMS: %.4f m/s²\n', ...
    mean(abs(error_raw)), rms(error_raw));
fprintf('LPF Error - Mean: %.4f m/s², RMS: %.4f m/s²\n', ...
    mean(abs(error_lpf)), rms(error_lpf));
fprintf('Improvement vs Raw: %.1f%% RMS reduction (KF)\n', ...
    100 * (1 - rms(error_kf)  / rms(error_raw)));
fprintf('Improvement vs Raw: %.1f%% RMS reduction (LPF)\n\n', ...
    100 * (1 - rms(error_lpf) / rms(error_raw)));

%% Plotting (Single Figure with Tabs)
hFig = figure('Name', 'Multi-Phase Adaptive PVA Kalman Filter Results', 'Position', [50, 50, 1400, 800]);
tab_group = uitabgroup('Parent', hFig);

% --- Position Tab ---
tab_pos = uitab(tab_group, 'Title', 'Position');

ax_pos_N = subplot(3,1,1, 'Parent', tab_pos);
hold(ax_pos_N, 'on'); grid(ax_pos_N, 'on');
plot(ax_pos_N, time_gps, ref_pos_N, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(ax_pos_N, time_gps, gps_pos_N, 'r.', 'MarkerSize', 4, 'DisplayName', 'GPS Meas');
plot(ax_pos_N, time_gps, pos_est(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'KF Est');
ylabel(ax_pos_N, 'North (m)'); title(ax_pos_N, 'Position Estimates');
legend(ax_pos_N, 'Location', 'best');

ax_pos_E = subplot(3,1,2, 'Parent', tab_pos);
hold(ax_pos_E, 'on'); grid(ax_pos_E, 'on');
plot(ax_pos_E, time_gps, ref_pos_E, 'k-', 'LineWidth', 1.5);
plot(ax_pos_E, time_gps, gps_pos_E, 'r.', 'MarkerSize', 4);
plot(ax_pos_E, time_gps, pos_est(:,2), 'b-', 'LineWidth', 1.2);
ylabel(ax_pos_E, 'East (m)');

ax_pos_D = subplot(3,1,3, 'Parent', tab_pos);
hold(ax_pos_D, 'on'); grid(ax_pos_D, 'on');
plot(ax_pos_D, time_gps, ref_pos_D, 'k-', 'LineWidth', 1.5);
plot(ax_pos_D, time_gps, gps_pos_D, 'r.', 'MarkerSize', 4);
plot(ax_pos_D, time_gps, pos_est(:,3), 'b-', 'LineWidth', 1.2);
ylabel(ax_pos_D, 'Down (m)'); xlabel(ax_pos_D, 'Time (s)');

% --- Velocity Tab ---
tab_vel = uitab(tab_group, 'Title', 'Velocity');

ax_vel_N = subplot(3,1,1, 'Parent', tab_vel);
hold(ax_vel_N, 'on'); grid(ax_vel_N, 'on');
plot(ax_vel_N, time_gps, ref_vel_N, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(ax_vel_N, time_gps, gps_vel_N, 'r.', 'MarkerSize', 4, 'DisplayName', 'GPS Meas');
plot(ax_vel_N, time_gps, vel_est(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'KF Est');
ylabel(ax_vel_N, 'North (m/s)'); title(ax_vel_N, 'Velocity Estimates');
legend(ax_vel_N, 'Location', 'best');

ax_vel_E = subplot(3,1,2, 'Parent', tab_vel);
hold(ax_vel_E, 'on'); grid(ax_vel_E, 'on');
plot(ax_vel_E, time_gps, ref_vel_E, 'k-', 'LineWidth', 1.5);
plot(ax_vel_E, time_gps, gps_vel_E, 'r.', 'MarkerSize', 4);
plot(ax_vel_E, time_gps, vel_est(:,2), 'b-', 'LineWidth', 1.2);
ylabel(ax_vel_E, 'East (m/s)');

ax_vel_D = subplot(3,1,3, 'Parent', tab_vel);
hold(ax_vel_D, 'on'); grid(ax_vel_D, 'on');
plot(ax_vel_D, time_gps, ref_vel_D, 'k-', 'LineWidth', 1.5);
plot(ax_vel_D, time_gps, gps_vel_D, 'r.', 'MarkerSize', 4);
plot(ax_vel_D, time_gps, vel_est(:,3), 'b-', 'LineWidth', 1.2);
ylabel(ax_vel_D, 'Down (m/s)'); xlabel(ax_vel_D, 'Time (s)');

% --- Acceleration Tab ---
tab_accel = uitab(tab_group, 'Title', 'Acceleration');

ax_accel_N = subplot(3,1,1, 'Parent', tab_accel);
hold(ax_accel_N, 'on'); grid(ax_accel_N, 'on');
plot(ax_accel_N, time_gps, ref_accel(:,1), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(ax_accel_N, time_gps, accel_raw(:,1), 'Color', [0.8, 0.4, 0.4], 'LineWidth', 1, 'DisplayName', 'Raw Deriv');
plot(ax_accel_N, time_gps, accel_lpf(:,1), 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.2, 'DisplayName', 'LPF Deriv');
plot(ax_accel_N, time_gps, accel_est(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'KF Est');
ylabel(ax_accel_N, 'North (m/s²)'); title(ax_accel_N, 'Acceleration Estimates');
legend(ax_accel_N, 'Location', 'best');

ax_accel_E = subplot(3,1,2, 'Parent', tab_accel);
hold(ax_accel_E, 'on'); grid(ax_accel_E, 'on');
plot(ax_accel_E, time_gps, ref_accel(:,2), 'k-', 'LineWidth', 1.5);
plot(ax_accel_E, time_gps, accel_raw(:,2), 'Color', [0.8, 0.4, 0.4], 'LineWidth', 1);
plot(ax_accel_E, time_gps, accel_lpf(:,2), 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.2);
plot(ax_accel_E, time_gps, accel_est(:,2), 'b-', 'LineWidth', 1.2);
ylabel(ax_accel_E, 'East (m/s²)');

ax_accel_D = subplot(3,1,3, 'Parent', tab_accel);
hold(ax_accel_D, 'on'); grid(ax_accel_D, 'on');
plot(ax_accel_D, time_gps, ref_accel(:,3), 'k-', 'LineWidth', 1.5);
plot(ax_accel_D, time_gps, accel_raw(:,3), 'Color', [0.8, 0.4, 0.4], 'LineWidth', 1);
plot(ax_accel_D, time_gps, accel_lpf(:,3), 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.2);
plot(ax_accel_D, time_gps, accel_est(:,3), 'b-', 'LineWidth', 1.2);
ylabel(ax_accel_D, 'Down (m/s²)'); xlabel(ax_accel_D, 'Time (s)');

% --- Magnitude & Error Tab ---
tab_mag = uitab(tab_group, 'Title', 'Magnitude & Error');

ax_mag = subplot(2,1,1, 'Parent', tab_mag);
hold(ax_mag, 'on'); grid(ax_mag, 'on');
plot(ax_mag, time_gps, ref_accel_mag, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(ax_mag, time_gps, accel_raw_mag, 'Color', [0.8, 0.4, 0.4], 'LineWidth', 1.2, 'DisplayName', 'Raw Derivative');
plot(ax_mag, time_gps, accel_lpf_mag, 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.2, 'DisplayName', 'LPF Derivative');
plot(ax_mag, time_gps, accel_est_mag, 'b-', 'LineWidth', 1.2, 'DisplayName', 'KF Estimate');
ylabel(ax_mag, 'Acceleration (m/s²)'); title(ax_mag, 'Acceleration Magnitude');
yyaxis(ax_mag, 'right');
plot(ax_mag, time_gps, active_sigma, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Active σ');
ylabel(ax_mag, 'Active σ_{accel}');
legend(ax_mag, 'Location', 'best');

ax_err = subplot(2,1,2, 'Parent', tab_mag);
hold(ax_err, 'on'); grid(ax_err, 'on');
plot(ax_err, time_gps, error_raw, 'Color', [0.8, 0.4, 0.4], 'LineWidth', 1.2, 'DisplayName', 'Raw Error');
plot(ax_err, time_gps, error_lpf, 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.2, 'DisplayName', 'LPF Error');
plot(ax_err, time_gps, error_kf, 'b-', 'LineWidth', 1.2, 'DisplayName', 'KF Error');
yline(ax_err, 0, 'k--', 'LineWidth', 1);
xlabel(ax_err, 'Time (s)'); ylabel(ax_err, 'Error (m/s²)'); title(ax_err, 'Acceleration Magnitude Error');
legend(ax_err, 'Location', 'best');

% --- Accel Uncertainty / Consistency Tab ---
tab_unc = uitab(tab_group, 'Title', 'Accel Uncertainty');

ax_unc_N = subplot(3,1,1, 'Parent', tab_unc);
hold(ax_unc_N, 'on'); grid(ax_unc_N, 'on');
plot(ax_unc_N, time_gps, accel_error(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Error a_N');
plot(ax_unc_N, time_gps,  sigma_accel(:,1), 'r--', 'LineWidth', 1,   'DisplayName', '+1\sigma');
plot(ax_unc_N, time_gps, -sigma_accel(:,1), 'r--', 'LineWidth', 1,   'DisplayName', '-1\sigma');
yline(ax_unc_N, 0, 'k:', 'LineWidth', 1);
ylabel(ax_unc_N, 'North (m/s²)');
title(ax_unc_N, 'Accel Error vs \pm1\sigma (North)');
legend(ax_unc_N, 'Location', 'best');

ax_unc_E = subplot(3,1,2, 'Parent', tab_unc);
hold(ax_unc_E, 'on'); grid(ax_unc_E, 'on');
plot(ax_unc_E, time_gps, accel_error(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Error a_E');
plot(ax_unc_E, time_gps,  sigma_accel(:,2), 'r--', 'LineWidth', 1,   'DisplayName', '+1\sigma');
plot(ax_unc_E, time_gps, -sigma_accel(:,2), 'r--', 'LineWidth', 1,   'DisplayName', '-1\sigma');
yline(ax_unc_E, 0, 'k:', 'LineWidth', 1);
ylabel(ax_unc_E, 'East (m/s²)');
title(ax_unc_E, 'Accel Error vs \pm1\sigma (East)');

ax_unc_D = subplot(3,1,3, 'Parent', tab_unc);
hold(ax_unc_D, 'on'); grid(ax_unc_D, 'on');
plot(ax_unc_D, time_gps, accel_error(:,3), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Error a_D');
plot(ax_unc_D, time_gps,  sigma_accel(:,3), 'r--', 'LineWidth', 1,   'DisplayName', '+1\sigma');
plot(ax_unc_D, time_gps, -sigma_accel(:,3), 'r--', 'LineWidth', 1,   'DisplayName', '-1\sigma');
yline(ax_unc_D, 0, 'k:', 'LineWidth', 1);
ylabel(ax_unc_D, 'Down (m/s²)');
xlabel(ax_unc_D, 'Time (s)');
title(ax_unc_D, 'Accel Error vs \pm1\sigma (Down)');

%% Save results
save('multi_phase_adaptive_pva_filter_results.mat', ...
     'time_gps', 'pos_est', 'vel_est', 'accel_est', ...
     'accel_raw', 'accel_lpf', 'ref_accel', 'vel_gps_filt', ...
     'gps_indices', 'active_sigma', 'P_accel', 'accel_error', 'sigma_accel');

fprintf('Results saved to multi_phase_adaptive_pva_filter_results.mat\n');
