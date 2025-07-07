%% 1. Setup: Define Initial State and Sweep Parameters
clear; clc; close all;

% --- Initial Rocket State at t = 66s (from LATEST debug printout) ---
% These values are the "baseline" or "original" state before adding weight.
V_base          = 397.95;     % Velocity (m/s)
rho_base        = 0.386;      % Air density (kg/m^3)
m_original      = 1218.07;    % Original vehicle mass (kg)
Iyy_original    = 2.29e+04;   % Original pitch inertia (kg*m^2)
T_base          = 26732.69;   % Thrust (N)
CNa_base        = 3.506;      % CN-alpha (/rad) - assumed constant with nose weight
CMa_original    = 19.160;     % Cm-alpha (/rad) at the original CG
CG_from_nose_original = 3.906;% Original CG from nose (m) - taken from L_arm value

% --- Fixed Geometric & Disturbance Parameters ---
S       = 0.200296;             % Reference Area (m^2)
d_ref   = sqrt(4*S/pi);         % Vehicle Diameter (m)
gimbal_from_nose = 9.542;       % Assumed physical location of gimbal from nose (m)
max_gimbal_angle_deg = 5.0;     % Actuator physical limit
max_alpha_for_trim_deg = 7.0;   % Maneuver requirement
wind_gust_mps = 9.0;
thrust_misalignment_deg = 0.25;
cg_offset_m = 0.05;

% --- Calculate the fixed Center of Pressure (CP) from the baseline state ---
% This ensures the trade study starts from a physically consistent state.
% Formula: CMa = CNa * (Xcp - Xcg) / d_ref
% Rearranged: Xcp = (CMa * d_ref / CNa) + Xcg
CP_from_nose = (CMa_original * d_ref / CNa_base) + CG_from_nose_original;
fprintf('Derived Center of Pressure (CP) location: %.3f m from nose.\n', CP_from_nose);

% --- Nose Weight Sweep Parameters ---
added_nose_weight = linspace(0, 500, 100); % Sweep from 0 to 500 kg of added weight
num_points = length(added_nose_weight);
fprintf('Performing trade study for added nose weight from %.1f to %.1f kg.\n\n', added_nose_weight(1), added_nose_weight(end));

%% 2. Perform the Sweep Calculation
% Pre-allocate arrays to store results
time_to_double_sweep = zeros(1, num_points);
total_gimbal_angle_sweep = zeros(1, num_points);
controllability_ratio_sweep = zeros(1, num_points);
static_margin_sweep = zeros(1, num_points);

% Constants for the loop
max_alpha_rad = deg2rad(max_alpha_for_trim_deg);
thrust_mis_rad = deg2rad(thrust_misalignment_deg);
q_bar = 0.5 * rho_base * V_base^2;

for i = 1:num_points
    % --- Step 1: Calculate New Mass Properties ---
    weight = added_nose_weight(i);
    m_new = m_original + weight;
    CG_from_nose_new = (m_original * CG_from_nose_original + weight * 0) / m_new; % Assuming nose weight is at X=0
    
    % New Moment of Inertia using Parallel Axis Theorem
    I_orig_term = Iyy_original + m_original * (CG_from_nose_original - CG_from_nose_new)^2;
    I_added_mass_term = weight * (CG_from_nose_new)^2; % I_cm for point mass is 0
    Iyy_new = I_orig_term + I_added_mass_term;
    
    L_arm_new = gimbal_from_nose - CG_from_nose_new;
    
    % --- Step 2: Calculate New Stability Derivatives ---
    static_margin_sweep(i) = (CG_from_nose_new - CP_from_nose) / d_ref;
    CMa_new = CNa_base * (CP_from_nose - CG_from_nose_new) / d_ref; % Manual CMa calculation

    M_alpha = (q_bar * S * d_ref / Iyy_new) * CMa_new;
    M_delta = (T_base * L_arm_new) / Iyy_new;
    
    % --- Step 3: Calculate Controllability Metrics ---
    if M_alpha > 0 % Unstable
        time_to_double_sweep(i) = log(2) / sqrt(M_alpha);
    else % Stable
        time_to_double_sweep(i) = inf;
    end
    
    if M_delta > 0
        trim_rad = -(M_alpha / M_delta) * max_alpha_rad;
        
        alpha_gust_rad = atan(wind_gust_mps / V_base);
        delta_wind_rad = abs(-(M_alpha / M_delta) * alpha_gust_rad);
        delta_thrust_mis_rad = thrust_mis_rad;
        delta_cg_offset_rad = abs(cg_offset_m / L_arm_new);
        
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
fprintf('Sweep calculation complete.\n');

% Print initial and final CG/Static Margin for comparison
fprintf('\n--- Results Summary ---\n');
fprintf('Initial Static Margin: %.2f calibers (Unstable) \n', static_margin_sweep(1));
fprintf('Final Static Margin (at %.0f kg): %.2f calibers (Stable) \n\n', added_nose_weight(end), static_margin_sweep(end));

%% 3. Plotting the Trade Study Results
fig = figure('Position', [100, 100, 1200, 1200]);
sgtitle(sprintf('Nose Weight Trade Study at t=%.1fs (V=%.0f m/s)', 66, V_base), ...
    'FontSize', 16, 'FontWeight', 'bold');

% Plot 1: Static Margin and Time to Double
ax1 = subplot(3, 1, 1);
yyaxis left;
plot(added_nose_weight, static_margin_sweep, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2.5);
hold on;
yline(0, '--', 'Neutral Stability', 'Color', 'k');
ylabel('Static Margin (calibers)', 'FontWeight', 'bold');
ax1.YColor = [0, 0.4470, 0.7410];
hold off;

yyaxis right;
plot(added_nose_weight, time_to_double_sweep, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2.5);
ylabel('Time to Double (s)', 'FontWeight', 'bold');
ax1.YColor = [0.8500, 0.3250, 0.0980];
ylim([0, 5]); % Cap y-axis for better visibility

title('Stability vs. Nose Weight', 'FontSize', 14, 'FontWeight', 'bold');
grid on; box on;

% Plot 2: Total Required Gimbal Angle
ax2 = subplot(3, 1, 2);
plot(added_nose_weight, total_gimbal_angle_sweep, '-', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 2.5);
hold on;
yline(max_gimbal_angle_deg, '-.', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', sprintf('Actuator Limit (%.1f°)', max_gimbal_angle_deg));
hold off;
title('Total Required Gimbal Angle vs. Nose Weight', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Gimbal Angle (degrees)');
legend('show', 'Location', 'northwest');
grid on; box on;
ylim([0, max(ylim)*1.1]);

% Plot 3: Controllability Ratio
ax3 = subplot(3, 1, 3);
plot(added_nose_weight, controllability_ratio_sweep, '-', 'Color', '#7E2F8E', 'LineWidth', 2.5);
hold on;
yline(2.0, '--', 'Recommended Design Goal (CR > 2.0)', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
yline(1.0, ':', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Absolute Physical Limit (CR > 1.0)');
hold off;
title('Controllability Ratio vs. Nose Weight', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Added Nose Weight (kg)', 'FontWeight', 'bold');
ylabel('Ratio');
legend('show', 'Location', 'northeast');
grid on; box on;
