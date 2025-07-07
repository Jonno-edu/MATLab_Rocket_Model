%% 1. Setup: Load Data and Define Parameters
clear; clc; close all;

% --- USER INPUTS ---
filename = 'simOut.mat'; 
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
paths.L_arm = 'CG_X'; 

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

%% 5. Debug Printout at a Specific Time Point
t_debug = 66; 

[~, debug_idx] = min(abs(t_linearize - t_debug));
actual_debug_time = t_linearize(debug_idx);

fprintf('\n\n--- DEBUG PRINTOUT AT t = %.1f s ---\n', actual_debug_time);

V_d = params.V(debug_idx); rho_d = params.rho(debug_idx); m_d = params.mass(debug_idx);
Iyy_d = params.I(debug_idx); T_d = params.T(debug_idx); CNa_d = params.CNa(debug_idx);
CMa_d = params.CMa(debug_idx); Cmq_d = params.Cmq(debug_idx); L_arm_d = params.L_arm(debug_idx);
q_bar_d = 0.5 * rho_d * V_d^2;

fprintf('1. Fetched Variables at t = %.1f s:\n', actual_debug_time);
fprintf('   V=%.2f m/s, rho=%.3f kg/m^3, m=%.2f kg, Iyy=%.2e kg*m^2, T=%.2f N\n', V_d, rho_d, m_d, Iyy_d, T_d);
fprintf('   CNa=%.3f, CMa=%.3f, Cmq=%.3f, L_arm=%.3f m, d_ref=%.3f m, q_bar=%.2f Pa\n\n', CNa_d, CMa_d, Cmq_d, L_arm_d, d_ref, q_bar_d);

fprintf('2. Dimensional Derivative Calculations:\n');
Ma_d = M_alpha(debug_idx); Md_d = M_delta(debug_idx);
fprintf('   M_alpha = (q_bar*S*d_ref/Iyy)*CMa = (%.2f*%.4f*%.3f/%.2e)*%.3f = %.4f\n', q_bar_d, S, d_ref, Iyy_d, CMa_d, Ma_d);
fprintf('   M_delta = (T*L_arm)/Iyy = (%.2f*%.3f)/%.2e = %.4f\n\n', T_d, L_arm_d, Iyy_d, Md_d);

% --- NEW: Controllability Debug Calculations ---
fprintf('3. Controllability Analysis Calculations:\n');
trim_rad_d = -(Ma_d / Md_d) * max_alpha_rad;
alpha_gust_rad_d = atan(wind_gust_mps / V_d);
delta_wind_rad_d = abs(-(Ma_d / Md_d) * alpha_gust_rad_d);
delta_thrust_mis_rad_d = thrust_mis_rad;
delta_cg_offset_rad_d = abs(cg_offset_m / L_arm_d);
stochastic_rss_rad_d = sqrt(delta_wind_rad_d^2 + delta_thrust_mis_rad_d^2 + delta_cg_offset_rad_d^2);
total_required_rad_d = abs(trim_rad_d) + stochastic_rss_rad_d;

fprintf('   a) Nominal Trim Angle:\n');
fprintf('      Formula: delta_trim = -(M_alpha / M_delta) * alpha_cmd\n');
fprintf('      Plugged in: -(%.4f / %.4f) * %.4f rad = %.4f rad (%.2f deg)\n\n', Ma_d, Md_d, max_alpha_rad, trim_rad_d, rad2deg(trim_rad_d));

fprintf('   b) Disturbance Angle Magnitudes:\n');
fprintf('      Wind Gust: |-(M_alpha/M_delta)*atan(gust/V)| = |-(%.4f/%.4f)*atan(%.1f/%.2f)| = %.4f rad (%.2f deg)\n', Ma_d, Md_d, wind_gust_mps, V_d, delta_wind_rad_d, rad2deg(delta_wind_rad_d));
fprintf('      Thrust Misalign: Constant = %.4f rad (%.2f deg)\n', delta_thrust_mis_rad_d, rad2deg(delta_thrust_mis_rad_d));
fprintf('      CG Offset: |cg_offset / L_arm| = |%.2f m / %.3f m| = %.4f rad (%.2f deg)\n\n', cg_offset_m, L_arm_d, delta_cg_offset_rad_d, rad2deg(delta_cg_offset_rad_d));

fprintf('   c) Total Disturbance Budget (RSS):\n');
fprintf('      Formula: budget = sqrt(d_wind^2 + d_thrust^2 + d_cg^2)\n');
fprintf('      Plugged in: sqrt(%.4f^2 + %.4f^2 + %.4f^2) = %.4f rad (%.2f deg)\n\n', delta_wind_rad_d, delta_thrust_mis_rad_d, delta_cg_offset_rad_d, stochastic_rss_rad_d, rad2deg(stochastic_rss_rad_d));

fprintf('   d) Total Required Angle:\n');
fprintf('      Formula: delta_total = |delta_trim| + budget\n');
fprintf('      Plugged in: |%.4f| + %.4f = %.4f rad (%.2f deg)\n\n', trim_rad_d, stochastic_rss_rad_d, total_required_rad_d, rad2deg(total_required_rad_d));

fprintf('   e) Final Controllability Ratios:\n');
fprintf('      CR (Nominal) = max_angle / |trim_angle| = %.1f deg / %.2f deg = %.2f\n', max_gimbal_angle_deg, abs(rad2deg(trim_rad_d)), controllability_ratio_no_disturb(debug_idx));
fprintf('      CR (Total)   = max_angle / total_angle = %.1f deg / %.2f deg = %.2f\n', max_gimbal_angle_deg, rad2deg(total_required_rad_d), controllability_ratio(debug_idx));

fprintf('--- END OF DEBUG PRINTOUT ---\n\n');


%% 6. Plotting the Results
% (Plotting section is unchanged)
fig = figure('Position', [100, 100, 1200, 1200]);
sgtitle(sprintf('TVC Rocket Stability & Controllability Analysis'), ...
    'FontSize', 16, 'FontWeight', 'bold');

% --- Plot 1: Time to Double ---
ax1 = subplot(3, 1, 1);
plot(t_linearize, time_to_double, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Time to Double');
hold on;
yline(0.5, '--', 'Challenging Threshold (0.5s)', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
[min_val, min_idx] = min(time_to_double);
text_str = sprintf('Min: %.2f s', min_val);
text(t_linearize(min_idx), min_val, text_str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title('Aerodynamic Instability', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Time (s)', 'FontWeight', 'bold');
legend('Location', 'northwest')
grid on; box on;

% --- Plot 2: Nozzle Angle Comparison ---
ax2 = subplot(3, 1, 2);
plot(t_linearize, trim_gimbal_angle_deg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 2, 'DisplayName', 'Nominal Trim Angle (Magnitude)');
hold on;
plot(t_linearize, total_gimbal_angle_deg, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2.5, 'DisplayName', 'Total Required Angle (Magnitude)');
yline(max_gimbal_angle_deg, '-.', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', sprintf('Actuator Limit (%.1f°)', max_gimbal_angle_deg));
[max_val, max_idx] = max(total_gimbal_angle_deg); 
text_str = sprintf('Max Required: %.2f°', max_val);
text(t_linearize(max_idx), max_val, text_str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title({'Nozzle Angle Requirement (Magnitudes)', ...
       sprintf('(Max AoA=%.1f°, Gust=%.1fm/s, CG Offset=%.2fm, Thrust Misalign=%.2f°)', ...
       max_alpha_for_trim_deg, wind_gust_mps, cg_offset_m, thrust_misalignment_deg)}, ...
       'FontSize', 14, 'FontWeight', 'bold');
ylabel('Gimbal Angle Mag. (deg)', 'FontWeight', 'bold');
legend('show', 'Location', 'northwest');
ylim([0, max(ylim)*1.1]);
grid on; box on;

% --- Plot 3: Controllability Ratio Comparison ---
ax3 = subplot(3, 1, 3);
plot(t_linearize, controllability_ratio, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Controllability Ratio');
hold on;
yline(2.0, '--', 'Recommended Design Goal (CR > 2.0)', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
yline(1.0, ':', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Absolute Physical Limit (CR > 1.0)');
[min_val, min_idx] = min(controllability_ratio);
text_str = sprintf('Min CR: %.2f', min_val);
text(t_linearize(min_idx), min_val, text_str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
hold off;
title({'Controllability Ratio (CR = Available Authority / Required Authority)', ...
       '(Includes all trim, disturbance, and offset effects)'}, ...
       'FontSize', 14, 'FontWeight', 'bold');
xlabel('Flight Time (s)', 'FontWeight', 'bold');
ylabel('Ratio', 'FontWeight', 'bold');
legend('Location', 'northeast');
grid on; box on;

% Link x-axes for all plots for synchronized zooming
linkaxes([ax1, ax2, ax3], 'x');
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
