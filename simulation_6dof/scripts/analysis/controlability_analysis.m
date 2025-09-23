%% 1. Setup: Load Data and Define Parameters
clear; clc; close all;

% --- USER INPUTS ---
filename = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'output', 'simOut.mat'); 
linearization_interval = 0.1; 
t_start_analysis = 1.0; 
max_alpha_for_trim_deg = 7;

% --- Disturbance and Imperfection Parameters (Industry Standards) ---
wind_gust_mps = 9.0;
thrust_misalignment_deg = 0.25;
cg_offset_m = 0.05;
max_gimbal_angle_deg = 5.0;

% --- Fixed Physical Parameters ---
S = 0.200296; % Reference Area (m^2)
d_ref = sqrt(4*S/pi); % Vehicle Diameter (m) as aero reference length
rocket_length = 9.542; % Total rocket length (m), nose to tail

% Load the simulation data
try
    load(filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

fprintf('Successfully loaded data from %s.\n', filename);
fprintf('Using vehicle diameter d_ref = %.3f m for moment calculations.\n', d_ref);

%% 2. Data Extraction and Interpolation
logsout_data = simOut.logsout;
paths.V = 'airspeed';
paths.rho = 'ENV.AirDensity';
paths.mass = 'mass';
paths.I = 'I';
paths.T = 'engineThrust';
paths.CNa = 'Cnalpha';
paths.CMa = 'Cmalpha';
paths.Cmq = 'cmq';
paths.L_arm = 'CG_X'; % CG from TAIL
paths.CP = 'CP';     % CP from NOSE

t_end = logsout_data.getElement(paths.V).Values.Time(end);
t_linearize = t_start_analysis:linearization_interval:floor(t_end);
num_points = length(t_linearize);
params = struct();

fields = fieldnames(paths);
for i = 1:length(fields)
    key = fields{i};
    path_str = paths.(key);
    try
        signal_obj = get_nested_signal(logsout_data, path_str);
        ts = signal_obj.Values;
        if isscalar(ts.Time)
            fprintf('Signal "%s" is constant. Propagating value.\n', path_str);
            if strcmp(key, 'I')
                params.(key) = repmat(squeeze(ts.Data(2,2,1)), size(t_linearize));
            else
                params.(key) = repmat(ts.Data(1), size(t_linearize));
            end
        else
            if strcmp(key, 'I')
                params.(key) = interp1(ts.Time, squeeze(ts.Data(2,2,:)), t_linearize, 'linear');
            else
                params.(key) = interp1(ts.Time, squeeze(ts.Data), t_linearize, 'linear');
            end
        end
    catch ME
        error('Failed to process signal "%s" at path "%s".\nError: %s', key, path_str, ME.message);
    end
end
fprintf('Data interpolation complete for %d time points, starting at t=%.1fs.\n', num_points, t_start_analysis);

%% 3. System Linearization at Each Time Point
Z_alpha = zeros(1, num_points);
Z_delta = zeros(1, num_points);
M_alpha = zeros(1, num_points);
M_q = zeros(1, num_points);
M_delta = zeros(1, num_points);
A_matrices = cell(1, num_points);
B_matrices = cell(1, num_points);

for i = 1:num_points
    V = params.V(i); rho = params.rho(i); m = params.mass(i); Iyy = params.I(i);
    T = params.T(i); CNa = params.CNa(i); CMa = params.CMa(i); Cmq = params.Cmq(i);
    L_arm_inst = params.L_arm(i);
    if V < 1.0, continue; end
    q_bar = 0.5 * rho * V^2;
    Z_alpha(i) = (q_bar * S / m) * CNa + T / m;
    Z_delta(i) = T / m;
    M_alpha(i) = (q_bar * S * d_ref / Iyy) * CMa;
    M_q(i)     = (q_bar * S * d_ref^2 / (4 * V * Iyy)) * Cmq;
    M_delta(i) = (T * L_arm_inst) / Iyy;
    A_matrices{i} = [ -Z_alpha(i)/V,  1,   0; M_alpha(i), M_q(i), 0; 0, 1, 0 ];
    B_matrices{i} = [ -Z_delta(i)/V; M_delta(i); 0 ];
end
fprintf('State-space linearization complete for all points.\n');

%% 4. Comprehensive Stability and Authority Analysis
% Calculate static margin
cg_from_nose = rocket_length - params.L_arm;
static_margin_calibers = (params.CP - cg_from_nose) / d_ref;

% Pre-allocate results
time_to_double = zeros(1, num_points);
trim_gimbal_angle_deg = zeros(1, num_points);
total_gimbal_angle_deg = zeros(1, num_points);
controllability_ratio = zeros(1, num_points);
controllability_ratio_no_disturb = zeros(1, num_points); 

max_alpha_rad = deg2rad(max_alpha_for_trim_deg);
thrust_mis_rad = deg2rad(thrust_misalignment_deg);

for i = 1:num_points
    if M_alpha(i) > 0
        time_to_double(i) = log(2) / sqrt(M_alpha(i));
    else
        time_to_double(i) = inf;
    end
    if M_delta(i) > 0 && params.V(i) > 1.0
        trim_rad = -(M_alpha(i) / M_delta(i)) * max_alpha_rad;
        trim_gimbal_angle_deg(i) = abs(rad2deg(trim_rad));
        alpha_gust_rad = atan(wind_gust_mps / params.V(i));
        delta_wind_rad = abs(-(M_alpha(i) / M_delta(i)) * alpha_gust_rad);
        delta_thrust_mis_rad = thrust_mis_rad;
        delta_cg_offset_rad = abs(cg_offset_m / params.L_arm(i));
        stochastic_rss_rad = sqrt(delta_wind_rad^2 + delta_thrust_mis_rad^2 + delta_cg_offset_rad^2);
        total_required_rad = abs(trim_rad) + stochastic_rss_rad;
        total_gimbal_angle_deg(i) = rad2deg(total_required_rad);
        if total_gimbal_angle_deg(i) > 1e-6
            controllability_ratio(i) = max_gimbal_angle_deg / total_gimbal_angle_deg(i);
        end
        if trim_gimbal_angle_deg(i) > 1e-6
            controllability_ratio_no_disturb(i) = max_gimbal_angle_deg / trim_gimbal_angle_deg(i);
        end
    end
end
fprintf('Comprehensive authority analysis complete.\n');

%% 5. Find Critical Time Points for Each Metric
% Find critical points (min/max) for each metric
[ttd_min_val, ttd_min_idx] = min(time_to_double);
[gimbal_max_val, gimbal_max_idx] = max(total_gimbal_angle_deg);
[cr_min_val, cr_min_idx] = min(controllability_ratio);
[sm_min_val, sm_min_idx] = min(static_margin_calibers);

% Get the corresponding timestamps
ttd_critical_time = t_linearize(ttd_min_idx);
gimbal_critical_time = t_linearize(gimbal_max_idx);
cr_critical_time = t_linearize(cr_min_idx);
sm_critical_time = t_linearize(sm_min_idx);

% Print critical points summary
fprintf('\n--- CRITICAL POINTS SUMMARY ---\n');
fprintf('Min Static Margin: %.2f calibers at t=%.1f s\n', sm_min_val, sm_critical_time);
fprintf('Most Unstable Point (Min Time to Double): %.2f s at t=%.1f s\n', ttd_min_val, ttd_critical_time);
fprintf('Max Authority Demand: %.2f deg at t=%.1f s\n', gimbal_max_val, gimbal_critical_time);
fprintf('Min Controllability Ratio: %.2f at t=%.1f s\n', cr_min_val, cr_critical_time);
fprintf('--- END CRITICAL POINTS ---\n\n');

%% 6. Enhanced Plotting with Critical Point Marking
fig = figure('Position', [100, 100, 1200, 1200]);
sgtitle(sprintf('TVC Rocket Stability & Controllability Analysis'), ...
    'FontSize', 16, 'FontWeight', 'bold');

% --- Plot 1: Static Margin ---
ax1 = subplot(4, 1, 1);
plot(t_linearize, static_margin_calibers, '-', 'Color', '#7E2F8E', 'LineWidth', 2.5, 'DisplayName', 'Static Margin');
hold on;
yline(0, '--k', 'LineWidth', 1.5, 'DisplayName', 'Neutral Stability');
plot(sm_critical_time, sm_min_val, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', sprintf('Min Margin: %.2f', sm_min_val));
text(sm_critical_time, sm_min_val, sprintf(' Min: %.2f\n t=%.1f s', sm_min_val, sm_critical_time), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title('Static Margin (Positive = Stable)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Margin (calibers)', 'FontWeight', 'bold');
legend('Location', 'best');
grid on; box on;
set(ax1, 'XTickLabel', []); % Remove x-axis tick labels

% --- Plot 2: Time to Double ---
ax2 = subplot(4, 1, 2);
plot(t_linearize, time_to_double, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Time to Double');
hold on;
yline(0.5, '--', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2, 'DisplayName', 'Challenging Threshold (0.5s)');
plot(ttd_critical_time, ttd_min_val, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', sprintf('Critical: %.2fs at t=%.1fs', ttd_min_val, ttd_critical_time));
text(ttd_critical_time, ttd_min_val, sprintf('  Min: %.2f s\n  t=%.1f s', ttd_min_val, ttd_critical_time), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title('Aerodynamic Instability', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Time (s)', 'FontWeight', 'bold');
legend('Location', 'northwest');
grid on; box on;
set(ax2, 'XTickLabel', []); % Remove x-axis tick labels

% --- Plot 3: Nozzle Angle Comparison ---
ax3 = subplot(4, 1, 3);
plot(t_linearize, trim_gimbal_angle_deg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 2, 'DisplayName', 'Nominal Trim Angle');
hold on;
plot(t_linearize, total_gimbal_angle_deg, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2.5, 'DisplayName', 'Total Required Angle');
yline(max_gimbal_angle_deg, '-.', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', sprintf('Actuator Limit (%.1f°)', max_gimbal_angle_deg));
plot(gimbal_critical_time, gimbal_max_val, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', sprintf('Critical: %.2f° at t=%.1fs', gimbal_max_val, gimbal_critical_time));
text(gimbal_critical_time, gimbal_max_val, sprintf('  Max: %.2f°\n  t=%.1f s', gimbal_max_val, gimbal_critical_time), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title({'Nozzle Angle Requirement', ...
       sprintf('(Max AoA=%.1f°, Gust=%.1fm/s, CG Offset=%.2fm, Thrust Misalign=%.2f°)', ...
       max_alpha_for_trim_deg, wind_gust_mps, cg_offset_m, thrust_misalignment_deg)}, ...
       'FontSize', 14, 'FontWeight', 'bold');
ylabel('Gimbal Angle (deg)', 'FontWeight', 'bold');
legend('show', 'Location', 'northwest');
ylim([0, max(ylim)*1.1]);
grid on; box on;
set(ax3, 'XTickLabel', []); % Remove x-axis tick labels

% --- Plot 4: Controllability Ratio ---
ax4 = subplot(4, 1, 4);
plot(t_linearize, controllability_ratio, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Controllability Ratio');
hold on;
yline(2.0, '--', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2, 'DisplayName', 'Design Goal (CR > 2.0)');
yline(1.0, ':', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Physical Limit (CR > 1.0)');
plot(cr_critical_time, cr_min_val, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', sprintf('Critical: %.2f at t=%.1fs', cr_min_val, cr_critical_time));
text(cr_critical_time, cr_min_val, sprintf('  Min: %.2f\n  t=%.1f s', cr_min_val, cr_critical_time), ...
     'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title({'Controllability Ratio (CR = Available Authority / Required Authority)', ...
       '(Includes all trim, disturbance, and offset effects)'}, ...
       'FontSize', 14, 'FontWeight', 'bold');
xlabel('Flight Time (s)', 'FontWeight', 'bold');
ylabel('Ratio', 'FontWeight', 'bold');
legend('show', 'Location', 'northeast');
grid on; box on;

% Link x-axes for all plots for synchronized zooming
linkaxes([ax1, ax2, ax3, ax4], 'x');
xlim([t_linearize(1), t_linearize(end)]);

%% Helper Function (unchanged)
function signal_obj = get_nested_signal(logsout_data, path_str)
    path_parts = strsplit(path_str, '.');
    try
        current_obj = logsout_data.getElement(path_parts{1});
        if length(path_parts) > 1
            current_struct = current_obj.Values;
            for k = 2:length(path_parts)
                current_struct = current_struct.(path_parts{k});
            end
            signal_obj.Values = current_struct;
        else
            signal_obj = current_obj;
        end
    catch ME
        rethrow(ME);
    end
end
