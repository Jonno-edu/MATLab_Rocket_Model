%% 1. Setup and Define Parameters
clear; clc; close all;

% Fixed system parameters
S = 0.200296;           % Reference area (m^2)
L = 9.542;              % Reference length (m)
L_arm = 3.9;            % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients
CLa = 2.0;
Cma = 0.885;            % Positive Cma indicates aerodynamic instability
Cmq_standard = -1.05;

% Time-varying flight data (excluding t=0 for cleaner plot)
time_vector = [5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 125, 135, 145];
velocity_vector = [150.6, 387.6, 728.5, 1604.7, 2523.6];
density_vector = [0.975, 0.4, 0.05, 0.0135, 0.00001];
mass_vector = [1528.1, 1228.4, 977.8, 628.3, 415.1];
thrust_vector = [2.48e4, 2.67e4, 2.75e4, 2.76e4, 2.76e4];
inertia_vector = [2.77e4, 2.31e4, 1.93e4, 1.36e4, 1.05e4];

% Analysis parameter: Max angle of attack (AoA)
max_alpha_deg = 7.0;
max_alpha_rad = deg2rad(max_alpha_deg);

%% 2. Perform Analysis at Each Point
num_points = length(time_vector);
time_to_double = zeros(1, num_points);
trim_gimbal_angle_deg = zeros(1, num_points);

for i = 1:num_points
    V = velocity_vector(i);
    rho = density_vector(i);
    m = mass_vector(i);
    T = thrust_vector(i);
    I = inertia_vector(i);
    
    % Calculate simplified constants (d-coefficients)
    d2 = T * L_arm / I;
    d3 = (rho * V^2 * S * L / (2 * I)) * Cma;
    
    % Calculate time to double
    if d3 > 0
        time_to_double(i) = log(2) / sqrt(d3);
    else
        time_to_double(i) = inf; 
    end
    
    % Calculate trim gimbal angle
    trim_gimbal_angle_rad = -(d3 / d2) * max_alpha_rad;
    trim_gimbal_angle_deg(i) = rad2deg(trim_gimbal_angle_rad);
end

%% 3. Generate Modern Plots
fig = figure('Position', [100, 100, 1100, 800]);
sgtitle('Rocket Controllability & Authority Profile', 'FontSize', 20, 'FontWeight', 'bold');

% Plot 1: Time to Double (Instability)
ax1 = subplot(2, 1, 1);
hold on;
plot(time_vector, time_to_double, 'o-', 'Color', [0.20 0.59 0.85], 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', [0.20 0.59 0.85], 'DisplayName', 'Time to Double');
yline(0.5, '--', 'Challenging Threshold (0.5s)', 'Color', [0.91 0.29 0.23], 'LineWidth', 2, 'LabelVerticalAlignment', 'bottom');

% Add text annotation for the minimum point
[min_val, min_idx] = min(time_to_double);
text_str = sprintf('Most Unstable Point\n%.2fs', min_val);
text(time_vector(min_idx), min_val, text_str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', 'BackgroundColor', [1 1 1 0.7]);
hold off;

title('Aerodynamic Instability vs. Time', 'FontSize', 15, 'FontWeight', 'bold');
ylabel('Time to Double (s)', 'FontSize', 12);
ylim([0, max(time_to_double(time_to_double < inf)) * 1.1]);
legend('Location', 'northwest', 'FontSize', 11);
grid on;
box on;

% Plot 2: Trim Gimbal Angle (Authority)
ax2 = subplot(2, 1, 2);
hold on;
plot(time_vector, trim_gimbal_angle_deg, 'o-', 'Color', [0.18 0.80 0.44], 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', [0.18 0.80 0.44], 'DisplayName', 'Required Gimbal Angle');

% --- FIX ---
% Draw actuator limit lines separately for robustness
yline(-5, ':', 'Color', [0.20 0.28 0.36], 'LineWidth', 2, 'HandleVisibility', 'off'); % Hide from legend
yline(5, ':', 'Typical Actuator Limit (±5°)', 'Color', [0.20 0.28 0.36], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');

% Add text annotation for the maximum demand point
[max_angle, max_idx] = min(trim_gimbal_angle_deg); % min because angles are negative
text_str = sprintf('Max Authority Demand\n%.2f°', abs(max_angle));
text(time_vector(max_idx), max_angle, text_str, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', 'BackgroundColor', [1 1 1 0.7]);
hold off;

title(sprintf('Required Trim Gimbal Angle for %.1f° AoA', max_alpha_deg), 'FontSize', 15, 'FontWeight', 'bold');
xlabel('Flight Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Gimbal Angle (degrees)', 'FontSize', 12);
legend('Location', 'southeast', 'FontSize', 11);
grid on;
box on;
