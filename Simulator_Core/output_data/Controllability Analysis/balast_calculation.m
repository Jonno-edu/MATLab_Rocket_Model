%% Nose Mass Trade Study at t=66s - Complete Script
clear; clc; close all;

%% 1. Setup and Load Data
filename = 'simOut.mat';
target_time = 66.0;  % Time point to analyze
linearization_interval = 0.1;  % For finding closest time point

try
    load(filename);
    fprintf('Successfully loaded data from %s.\n', filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

%% 2. Extract Baseline State at t=66s
logsout_data = simOut.logsout;

% Define signal paths
paths = struct();
paths.V = 'airspeed';
paths.rho = 'ENV.AirDensity'; 
paths.mass = 'mass';
paths.I = 'I';
paths.T = 'engineThrust';
paths.CNa = 'Cnalpha';
paths.CMa = 'Cmalpha';
paths.Cmq = 'cmq';     % Added Cmq for pitch damping
paths.L_arm = 'CG_X';  % CG distance from TAIL
paths.CP = 'CP';       % CP distance from NOSE

% Extract values at t=66s
baseline_state = struct();
field_names = fieldnames(paths);
for i = 1:length(field_names)
    signal_name = field_names{i};
    signal_path = paths.(signal_name);
    
    try
        signal_obj = get_nested_signal(logsout_data, signal_path);
        ts = signal_obj.Values;
        
        if isscalar(ts.Time)
            fprintf('Signal "%s" is constant. Using constant value.\n', signal_path);
            if strcmp(signal_name, 'I')
                baseline_state.(signal_name) = squeeze(ts.Data(2,2,1));
            else
                baseline_state.(signal_name) = ts.Data(1);
            end
        else
            if strcmp(signal_name, 'I')
                inertia_data = squeeze(ts.Data(2,2,:));
                baseline_state.(signal_name) = interp1(ts.Time, inertia_data, target_time);
            else
                baseline_state.(signal_name) = interp1(ts.Time, squeeze(ts.Data), target_time);
            end
        end
    catch ME
        error('Failed to extract signal "%s": %s', signal_path, ME.message);
    end
end

%% 3. Define Fixed Parameters and Print State
% Physical parameters
S = 0.200296;  % Reference area (m^2)
d_ref = sqrt(4*S/pi);  % Vehicle diameter (m)
rocket_length = 9.542;  % Total rocket length (m)

% Coordinate system clarification
baseline_cg_from_tail = baseline_state.L_arm;
baseline_cg_from_nose = rocket_length - baseline_cg_from_tail;
CP_from_nose = baseline_state.CP;

% Disturbance parameters
wind_gust_mps = 9.0;
thrust_misalignment_deg = 0.25;
cg_offset_m = 0.05;
max_gimbal_angle_deg = 5.0;
max_alpha_for_trim_deg = 7.0;

% Print baseline state (Original)
fprintf('\n=== BASELINE STATE AT t=%.1fs ===\n', target_time);
fprintf('Mass: %.2f kg\n', baseline_state.mass);
fprintf('CG from tail (CG_X): %.3f m\n', baseline_cg_from_tail);
fprintf('CG from nose: %.3f m\n', baseline_cg_from_nose);
fprintf('CP from nose: %.3f m\n', CP_from_nose);
fprintf('Velocity: %.2f m/s\n', baseline_state.V);
fprintf('Air density: %.3f kg/m³\n', baseline_state.rho);
fprintf('Thrust: %.2f N\n', baseline_state.T);
fprintf('CNa: %.3f /rad\n', baseline_state.CNa);
fprintf('CMa: %.3f /rad\n', baseline_state.CMa);
fprintf('Cmq: %.3f /rad\n', baseline_state.Cmq);
fprintf('Inertia: %.2e kg·m²\n', baseline_state.I);
fprintf('Static margin: %.3f calibers\n', (CP_from_nose - baseline_cg_from_nose) / d_ref);
fprintf('================================\n\n');

% --- NEW: Print Plant Parameters in Requested Format ---
fprintf('%% Plant Parameters (TVC Rocket @ t=%.0fs)\n', target_time);
fprintf('%% Physical parameters (SI units)\n');
fprintf('m = %.1f;             %% rocket mass (kg)\n', baseline_state.mass);
fprintf('rho = %.1f;              %% air density (kg/m³)\n', baseline_state.rho);
fprintf('V = %.1f;              %% velocity (m/s)\n', baseline_state.V);
fprintf('S = %.6f;           %% reference area (m²)\n', S);
fprintf('L = %.3f;              %% reference length (m)\n', rocket_length);
fprintf('I = %.2e;             %% moment of inertia about pitch axis (kg⋅m²)\n', baseline_state.I);
fprintf('T = %.2e;             %% thrust (N)\n', baseline_state.T);
fprintf('L_arm = %.1f;            %% TVC moment arm from CG to nozzle (m)\n', baseline_cg_from_tail);
fprintf('\n%% Aerodynamic coefficients\n');
fprintf('CNa = %.1f;              %% Normal force coefficient derivative (1/rad), equivalent to CLa for small alpha\n', baseline_state.CNa);
fprintf('Cma = %.3f;            %% Pitching moment coefficient derivative (1/rad) - positive = unstable\n', baseline_state.CMa);
if isfield(baseline_state, 'Cmq')
    fprintf('Cmq = %.2f;            %% Pitch damping derivative (1/rad)\n', baseline_state.Cmq);
else
    fprintf('%% Cmq not found in data.\n');
end
fprintf('\n');
% ----------------------------------------------------

%% 4. Nose Mass Sweep Setup
added_mass_range = linspace(0, 500, 100);  % 0 to 500 kg
num_points = length(added_mass_range);

% Pre-allocate result arrays
time_to_double_sweep = zeros(1, num_points);
total_gimbal_angle_sweep = zeros(1, num_points);
controllability_ratio_sweep = zeros(1, num_points);
static_margin_sweep = zeros(1, num_points);
new_cg_from_nose_positions = zeros(1, num_points);
new_cg_from_tail_positions = zeros(1, num_points);

% Convert angles to radians
max_alpha_rad = deg2rad(max_alpha_for_trim_deg);
thrust_mis_rad = deg2rad(thrust_misalignment_deg);

% Calculate dynamic pressure
q_bar = 0.5 * baseline_state.rho * baseline_state.V^2;

%% 5. Perform Mass Sweep Analysis
fprintf('Performing nose mass sweep analysis...\n');
for i = 1:num_points
    added_mass = added_mass_range(i);
    
    % === Calculate New Mass Properties ===
    new_total_mass = baseline_state.mass + added_mass;
    
    % New CG position (nose mass added at x=0 from nose)
    new_cg_from_nose = (baseline_state.mass * baseline_cg_from_nose + added_mass * 0) / new_total_mass;
    new_cg_from_tail = rocket_length - new_cg_from_nose;
    
    new_cg_from_nose_positions(i) = new_cg_from_nose;
    new_cg_from_tail_positions(i) = new_cg_from_tail;
    
    % New moment of inertia using parallel axis theorem
    cg_shift = baseline_cg_from_nose - new_cg_from_nose;
    I_original_term = baseline_state.I + baseline_state.mass * cg_shift^2;
    I_added_term = added_mass * (new_cg_from_nose - 0)^2;
    new_inertia = I_original_term + I_added_term;
    
    % TVC moment arm is distance from CG to nozzle (at tail)
    new_L_arm_tvc = new_cg_from_tail;
    
    % === Calculate New Aerodynamic Properties ===
    % Static margin: (CP - CG) / d_ref. Positive = stable.
    static_margin_sweep(i) = (CP_from_nose - new_cg_from_nose) / d_ref;
    
    % New CMa based on new CG position (CP stays constant)
    new_CMa = baseline_state.CNa * (new_cg_from_nose - CP_from_nose) / d_ref;
    
    % === Calculate Dimensional Derivatives ===
    % M_alpha is the dimensional derivative, which already includes inertia.
    M_alpha = (q_bar * S * d_ref / new_inertia) * new_CMa; 
    M_delta = (baseline_state.T * new_L_arm_tvc) / new_inertia;
    
    % === Stability Analysis: Time to Double ===
    % This calculation is valid for an unstable vehicle (M_alpha > 0).
    if M_alpha > 0
        time_to_double_sweep(i) = log(2) / sqrt(M_alpha);
    else
        time_to_double_sweep(i) = inf;  % Vehicle is stable
    end
    
    % === Authority Analysis ===
    if M_delta > 0
        trim_rad = -(M_alpha / M_delta) * max_alpha_rad;
        alpha_gust_rad = atan(wind_gust_mps / baseline_state.V);
        delta_wind_rad = abs(-(M_alpha / M_delta) * alpha_gust_rad);
        delta_thrust_mis_rad = thrust_mis_rad;
        delta_cg_offset_rad = abs(cg_offset_m / new_L_arm_tvc);
        stochastic_rss_rad = sqrt(delta_wind_rad^2 + delta_thrust_mis_rad^2 + delta_cg_offset_rad^2);
        total_required_rad = abs(trim_rad) + stochastic_rss_rad;
        total_gimbal_angle_sweep(i) = rad2deg(total_required_rad);
        
        if total_gimbal_angle_sweep(i) > 1e-6
            controllability_ratio_sweep(i) = max_gimbal_angle_deg / total_gimbal_angle_sweep(i);
        else
            controllability_ratio_sweep(i) = inf;
        end
    end
end
fprintf('Analysis complete for %d mass points.\n\n', num_points);

%% 6. Find Critical Points and Transitions
stability_transition_idx = find(static_margin_sweep >= 0, 1, 'first');
if ~isempty(stability_transition_idx)
    transition_mass = added_mass_range(stability_transition_idx);
    fprintf('=== STABILITY TRANSITION ===\n');
    fprintf('Vehicle becomes statically stable at %.1f kg added nose mass\n', transition_mass);
    fprintf('New CG position: %.3f m from nose (%.3f m from tail)\n', ...
        new_cg_from_nose_positions(stability_transition_idx), ...
        new_cg_from_tail_positions(stability_transition_idx));
    fprintf('===========================\n\n');
else
    fprintf('=== NO STABILITY TRANSITION FOUND ===\n');
end

finite_ttd = time_to_double_sweep(~isinf(time_to_double_sweep));
if ~isempty(finite_ttd)
    [min_ttd, min_ttd_idx] = min(time_to_double_sweep);
    fprintf('Most unstable (min time to double): %.3f s at %.1f kg\n', min_ttd, added_mass_range(min_ttd_idx));
end

[max_gimbal, max_gimbal_idx] = max(total_gimbal_angle_sweep);
[min_cr, min_cr_idx] = min(controllability_ratio_sweep);
fprintf('=== CRITICAL POINTS ===\n');
fprintf('Max authority demand: %.2f° at %.1f kg\n', max_gimbal, added_mass_range(max_gimbal_idx));
fprintf('Min controllability ratio: %.2f at %.1f kg\n', min_cr, added_mass_range(min_cr_idx));
fprintf('=====================\n\n');

%% 7. Create Plots as Requested
% Figure 1: Stability and Controllability Metrics (Stacked)
fig1 = figure('Position', [100, 100, 1000, 800]);
sgtitle(sprintf('Stability & Controllability Metrics at t=%.1fs (V=%.0f m/s)', ...
    target_time, baseline_state.V), 'FontSize', 16, 'FontWeight', 'bold');

% --- Custom Subplot Layout ---
left_margin = 0.1;
right_margin = 0.05;
bottom_margin = 0.1;
top_margin = 0.1;
v_gap = 0.08;
plot_width = 1 - left_margin - right_margin;
rel_heights = [1, 0.7, 1, 1]; 
total_rel_height = sum(rel_heights);
total_v_space = 1 - bottom_margin - top_margin - (length(rel_heights) - 1) * v_gap;
abs_heights = (rel_heights / total_rel_height) * total_v_space;
b4 = bottom_margin;
b3 = b4 + abs_heights(4) + v_gap;
b2 = b3 + abs_heights(3) + v_gap;
b1 = b2 + abs_heights(2) + v_gap;
pos1 = [left_margin, b1, plot_width, abs_heights(1)];
pos2 = [left_margin, b2, plot_width, abs_heights(2)];
pos3 = [left_margin, b3, plot_width, abs_heights(3)];
pos4 = [left_margin, b4, plot_width, abs_heights(4)];

% Plot 1: Static Margin
ax1 = subplot('Position', pos1);
plot(added_mass_range, static_margin_sweep, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2, 'DisplayName', 'Static Margin');
hold on;
yline(0, '--k', 'LineWidth', 1.5, 'DisplayName', 'Neutral Stability');
if ~isempty(stability_transition_idx)
    xline(transition_mass, ':', 'Color', 'g', 'LineWidth', 2, 'DisplayName', 'Stability Transition');
end
title('Static Margin', 'FontSize', 12);
ylabel('Margin (calibers)');
grid on; box on;
legend('Location', 'best');
set(ax1, 'XTickLabel', []);
hold off;

% Plot 2: Time to Double
ax2 = subplot('Position', pos2);
plot(added_mass_range, time_to_double_sweep, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2, 'DisplayName', 'Time to Double');
hold on;
yline(0.5, '--', 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth', 2, 'DisplayName', 'Critical Threshold (0.5s)');
if ~isempty(finite_ttd)
    ylim([0, min(5, max(finite_ttd)*1.1)]);
else
    ylim([0, 5]);
end
title('Time to Double', 'FontSize', 12);
ylabel('Time (s)');
grid on; box on;
legend('Location', 'best');
set(ax2, 'XTickLabel', []);
hold off;

% Plot 3: Total Required Gimbal Angle
ax3 = subplot('Position', pos3);
plot(added_mass_range, total_gimbal_angle_sweep, '-', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 2, 'DisplayName', 'Required Angle');
hold on;
yline(max_gimbal_angle_deg, '--r', 'LineWidth', 2, 'DisplayName', sprintf('Actuator Limit (%.1f°)', max_gimbal_angle_deg));
title('Total Required Gimbal Angle', 'FontSize', 12);
ylabel('Angle (degrees)');
grid on; box on;
legend('Location', 'best');
set(ax3, 'XTickLabel', []);
hold off;

% Plot 4: Controllability Ratio
ax4 = subplot('Position', pos4);
plot(added_mass_range, controllability_ratio_sweep, '-', 'Color', '#7E2F8E', 'LineWidth', 2, 'DisplayName', 'Controllability Ratio');
hold on;
yline(2.0, '--', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2, 'DisplayName', 'Design Goal (>2.0)');
yline(1.0, ':k', 'LineWidth', 2, 'DisplayName', 'Physical Limit (>1.0)');
plot(added_mass_range(min_cr_idx), min_cr, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Min Ratio');
title('Controllability Ratio', 'FontSize', 12);
xlabel('Added Nose Mass (kg)');
ylabel('Ratio');
grid on; box on;
legend('Location', 'best');
hold off;

% Figure 2: Rocket Configuration
fig2 = figure('Position', [1150, 100, 600, 500]);
sgtitle(sprintf('Rocket Configuration vs. Added Mass (t=%.1fs)\n\n', target_time), ...
    'FontSize', 16, 'FontWeight', 'bold');

plot(added_mass_range, new_cg_from_tail_positions, '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'CG Position');
hold on;
CP_from_tail = rocket_length - CP_from_nose;
yline(CP_from_tail, '--r', 'LineWidth', 2, 'DisplayName', sprintf('CP Position (%.2f m)', CP_from_tail));
yline(baseline_cg_from_tail, ':b', 'LineWidth', 2, 'DisplayName', sprintf('Original CG (%.2f m)', baseline_cg_from_tail));
if ~isempty(stability_transition_idx)
    xline(transition_mass, ':', 'Color', 'g', 'LineWidth', 2, 'DisplayName', 'Stability Transition');
    plot(transition_mass, new_cg_from_tail_positions(stability_transition_idx), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Stable CG');
end
hold off;
title({'Coordinates (from Tail)'}, 'FontSize', 12);
xlabel('Added Nose Mass (kg)');
ylabel('Distance from Tail (m)');
ylim([0, rocket_length]);
legend('Location', 'best');
grid on; box on;

fprintf('Trade study plots generated successfully.\n');

%% Helper Function
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
