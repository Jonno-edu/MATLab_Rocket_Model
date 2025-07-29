% STEVE Rocket Simulation Runner and Visualizer
% This script runs the complete rocket simulation workflow:
% 1. Initializes all parameters
% 2. Sets up the control system
% 3. Executes the Simulink model
% 4. Visualizes and analyzes the results
% 5. Exports data for external visualization tools

clear; clc; close all;

% Get the project root directory and add required paths
scriptPath = mfilename('fullpath');
[scriptDir, ~, ~] = fileparts(scriptPath);
projectRoot = fileparts(fileparts(scriptDir)); % Two levels up from script

% Add required paths (relative to project root)
addpath(fullfile(projectRoot, 'Simulator_Core'));
addpath(fullfile(projectRoot, 'MATLAB_Toolbox'));

% Initialize parameters and design control system
disp('Initializing parameters...');
initialize_parameters;
disp('Setting up control system...');
ControlSystemDesign;

% Load and run the Simulink model using full path
mdl = "STEVE_Simulation";
modelPath = fullfile(projectRoot, 'Simulator_Core', 'STEVE_Simulation.slx');
if ~exist(modelPath, 'file')
    error('Cannot find Simulink model at: %s', modelPath);
end
load_system(modelPath);
out = sim(mdl);

% Extract time vector and data
t = out.tout;
Ve = out.Ve.Data;
Xe = out.Xe.Data;
Vb = out.Vb.Data;
phi = out.phi.Data;
theta = out.theta.Data;
psi = out.psi.Data;
alpha = out.alpha.Data;

% Extract force and moment data
Fwind = out.Fwind.Data;
Mwind = out.Mwind.Data;
Fthrust = out.Fthrust.Data;
Mthrust = out.Mthrust.Data;
Fnet = out.Fnet.Data;
Mnet = out.Mnet.Data;

% Skip the first N frames
N = 1;  % Number of frames to skip
t = t(N+1:end) - Sim.Timestep * N;
Ve = Ve(N+1:end,:);
Xe = Xe(N+1:end,:);
Vb = Vb(N+1:end,:);
phi = phi(N+1:end);
theta = theta(N+1:end);
psi = psi(N+1:end);
alpha = alpha(N+1:end);

% Skip first N frames for forces and moments too
Fwind = Fwind(N+1:end,:);
Mwind = Mwind(N+1:end,:);
Fthrust = Fthrust(N+1:end,:);
Mthrust = Mthrust(N+1:end,:);
Fnet = Fnet(N+1:end,:);
Mnet = Mnet(N+1:end,:);

% Convert earth frame z to make "up" positive
Xe(:,3) = -Xe(:,3);  % Invert z-position (altitude)
Ve(:,3) = -Ve(:,3);  % Invert z-velocity

% Create main figure with tabs
fig = figure('Name', 'Rocket Simulation Results', 'Position', [100, 100, 1200, 700]);
tabgp = uitabgroup(fig);

% Tab 1: Position
tab1 = uitab(tabgp, 'Title', 'Position');

% 3D trajectory
axes('Parent', tab1, 'Position', [0.05, 0.55, 0.3, 0.35]);
plot3(Xe(:,1), Xe(:,2), Xe(:,3), 'b-', 'LineWidth', 2);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory');

% Altitude vs Time (vertical component separate)
axes('Parent', tab1, 'Position', [0.4, 0.55, 0.55, 0.35]);
plot(t, Xe(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude vs Time');

% Ground Track (horizontal plane)
axes('Parent', tab1, 'Position', [0.05, 0.1, 0.3, 0.35]);
plot(Xe(:,1), Xe(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('X (m)'); ylabel('Y (m)');
title('Ground Track (X-Y Plane)');

% Horizontal position components
axes('Parent', tab1, 'Position', [0.4, 0.1, 0.55, 0.35]);
plot(t, Xe(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Xe(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Position (m)');
title('Horizontal Position Components vs Time');
legend('X', 'Y', 'Location', 'best');

% Tab 2: Velocity
tab2 = uitab(tabgp, 'Title', 'Velocity');

% Earth-frame horizontal velocity components
axes('Parent', tab2, 'Position', [0.05, 0.55, 0.4, 0.35]);
plot(t, Ve(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Ve(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Earth-Frame Horizontal Velocity');
legend('V_x', 'V_y', 'Location', 'best');

% Earth-frame vertical velocity component (separate)
axes('Parent', tab2, 'Position', [0.55, 0.55, 0.4, 0.35]);
plot(t, Ve(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Earth-Frame Vertical Velocity (Z)');

% Body-frame velocities (leave as is)
axes('Parent', tab2, 'Position', [0.05, 0.1, 0.4, 0.35]);
plot(t, Vb(:,1), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Body-Frame Axial Velocity (X)');

% Body-frame lateral velocity components
axes('Parent', tab2, 'Position', [0.55, 0.1, 0.4, 0.35]);
plot(t, Vb(:,2), 'g-', 'LineWidth', 2); hold on;
plot(t, Vb(:,3), 'b-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Body-Frame Lateral Velocity Components');
legend('V_b_y', 'V_b_z', 'Location', 'best');

% Tab 3: Attitude and Angular Rates
tab3 = uitab(tabgp, 'Title', 'Attitude');

% Extract angular rates
w = out.w.Data;
w = w(N+1:end,:);  % Skip first N frames like other variables

% Extract Nozzle Angle
nozzleAngle = out.NozzleAngle.Data;
nozzleAngle = nozzleAngle(N+1:end); 

plot_height = 0.25;
vertical_gap = 0.07;

% Nozzle Angle (bottom plot)
axes('Parent', tab3, 'Position', [0.1, 0.05, 0.8, plot_height]);
plot(t, nozzleAngle*180/pi, 'b-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Nozzle Deflection Angle');

% Pitch Rate (middle plot)
axes('Parent', tab3, 'Position', [0.1, 0.37, 0.8, plot_height]);
plot(t, w(:,2)*180/pi, 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Pitch Rate (omega_y)');

% Pitch Angle and Angle of Attack (top plot)
axes('Parent', tab3, 'Position', [0.1, 0.69, 0.8, plot_height]);
plot(t, theta*180/pi, 'g-', 'LineWidth', 2);
hold on;
plot(t, alpha*180/pi, 'm--', 'LineWidth', 2);
hold off;
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle (theta) and Angle of Attack');
legend('Pitch Angle', 'Angle of Attack', 'Location', 'best');



% Calculate and display key flight metrics
tab4 = uitab(tabgp, 'Title', 'Flight Metrics');

% Calculate metrics
max_altitude = max(Xe(:,3));
[max_speed, max_speed_idx] = max(sqrt(sum(Ve.^2, 2)));
horiz_dist = sqrt(Xe(end,1)^2 + Xe(end,2)^2);
max_horiz_dist = max(sqrt(sum(Xe(:,1:2).^2, 2)));
flight_time = t(end);
[max_accel, max_accel_idx] = max(diff(sqrt(sum(Ve.^2, 2)))./diff(t));
max_aoa = max(abs(alpha)) * 180/pi;

burnout_time = Actuators.Engine.BurnTime;
% Find the index of the time closest to burnout time
[~, burnout_idx] = min(abs(t - burnout_time));
burnout_altitude = Xe(burnout_idx,3);
burnout_speed = sqrt(sum(Ve(burnout_idx,:).^2));
burnout_horiz_dist = sqrt(Xe(burnout_idx,1)^2 + Xe(burnout_idx,2)^2);

% Create text display for metrics
metrics_text = {
    sprintf('Maximum Altitude: %.1f km', max_altitude/1000.0)
    sprintf('Maximum Speed: %.1f m/s (at t=%.1f s)', max_speed, t(max_speed_idx))
    sprintf('Final Distance from Launch Pad: %.1f km', horiz_dist/1000.0)
    sprintf('Maximum Distance from Launch Pad: %.1f km', max_horiz_dist/1000.0)
    sprintf('Total Flight Time: %.1f s', flight_time)
    sprintf('Maximum Acceleration: %.1f m/s²', max_accel)
    sprintf('Maximum Angle of Attack: %.1f degrees', max_aoa)
    sprintf('Burnout Time: %.1f s', burnout_time)
    sprintf('Burnout Altitude: %.1f m', burnout_altitude)
    sprintf('Burnout Speed: %.1f m/s', burnout_speed)
    sprintf('Burnout Horizontal Distance: %.1f m', burnout_horiz_dist)
};

% Display metrics in a clean text box
annotation(tab4, 'textbox', [0.1 0.1 0.8 0.8], ...
    'String', metrics_text, ...
    'FontSize', 14, ...
    'FontName', 'Consolas', ...
    'EdgeColor', 'none', ...
    'VerticalAlignment', 'middle');

% Tab 5: Forces
tab5 = uitab(tabgp, 'Title', 'Forces');

% Axial Forces (Fx)
axes('Parent', tab5, 'Position', [0.05, 0.55, 0.4, 0.35]);
plot(t, Fwind(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Fthrust(:,1), 'r-', 'LineWidth', 2);
plot(t, Fnet(:,1), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Axial Forces (F_x)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Normal Forces (Fz)
axes('Parent', tab5, 'Position', [0.55, 0.55, 0.4, 0.35]);
plot(t, Fwind(:,3), 'b-', 'LineWidth', 2); hold on;
plot(t, Fthrust(:,3), 'r-', 'LineWidth', 2);
plot(t, Fnet(:,3), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Normal Forces (F_z)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Force Magnitudes
axes('Parent', tab5, 'Position', [0.05, 0.1, 0.4, 0.35]);
plot(t, sqrt(Fwind(:,1).^2 + Fwind(:,3).^2), 'b-', 'LineWidth', 2); hold on;
plot(t, sqrt(Fthrust(:,1).^2 + Fthrust(:,3).^2), 'r-', 'LineWidth', 2);
plot(t, sqrt(Fnet(:,1).^2 + Fnet(:,3).^2), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Force Magnitudes');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Tab 6: Moments
tab6 = uitab(tabgp, 'Title', 'Moments');

% Pitching Moments (My)
axes('Parent', tab6, 'Position', [0.1, 0.55, 0.8, 0.35]);
plot(t, Mwind(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t, Mthrust(:,2), 'r-', 'LineWidth', 2);
plot(t, Mnet(:,2), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Moment (N·m)');
title('Pitching Moments (M_y)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Print metrics to command window as well
fprintf('\n=== Key Flight Metrics ===\n');
fprintf('%s\n', metrics_text{:});

% Tab 7: Wind Velocity
tab7 = uitab(tabgp, 'Title', 'Wind Velocity');

% Extract wind velocity data
windVelocity = out.windVelocity.Data;

% Skip the first N frames as with other data
windVelocity = windVelocity(N+1:end,:);

% Plot wind velocity components
axes('Parent', tab7, 'Position', [0.1, 0.55, 0.8, 0.35]);
plot(t, windVelocity(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, windVelocity(:,2), 'g-', 'LineWidth', 2);
plot(t, windVelocity(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Wind Velocity Components');
legend('V_x', 'V_y', 'V_z', 'Location', 'best');

% Plot wind speed magnitude
axes('Parent', tab7, 'Position', [0.1, 0.1, 0.8, 0.35]);
windSpeed = sqrt(sum(windVelocity.^2, 2));
plot(t, windSpeed, 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Wind Speed Magnitude');



% --- START: Updated Data Export Section ---

% Save trajectory data for Python visualization
sim_fps = 1/Sim.Timestep;          % Original simulation FPS
target_fps = 100;                   % Desired output FPS
downsample_factor = round(sim_fps/target_fps);

% Downsample data for 100 FPS
sampled_indices = 1:downsample_factor:length(t);

% Interpolate CG data (Assuming COM_X is CG distance from AFT end)
sampled_times = t(sampled_indices);
cg_from_aft_m = interp1(MassData.Time, MassData.COM_X, sampled_times, 'linear', 'extrap');

% Calculate CG position relative to the NOSE in body frame
% Assumes body X-axis points forward from the nose.
rocket_length = RocketAeroPhysical.Length; % Ensure this variable exists
cg_body_x_m = rocket_length - cg_from_aft_m; % Distance from nose along body X

% Assuming CG offset is only along the body X-axis in this 3DOF sim
cg_body_y_m = zeros(size(cg_body_x_m));
cg_body_z_m = zeros(size(cg_body_x_m));

% Get sampled position, pitch, and nozzle angle
pos_xyz_m = Xe(sampled_indices,:);         % World frame [X, Y, Z_up]
pitch_rad = theta(sampled_indices);        % Pitch angle (radians)
nozzle_angle_rad = nozzleAngle(sampled_indices); % Nozzle angle (radians)

% Combine data for export (Focus on data needed for Blender animation)
trajectory_data = [t(sampled_indices), ...
                   pos_xyz_m, ...             % World Pos X, Y, Z
                   pitch_rad, ...             % Body Pitch
                   cg_body_x_m, ...           % CG X relative to nose
                   cg_body_y_m, ...           % CG Y relative to nose
                   cg_body_z_m, ...           % CG Z relative to nose
                   nozzle_angle_rad];         % Nozzle Angle

% Create table with explicit headers for Blender script
trajectory_table = array2table(trajectory_data, ...
    'VariableNames', {'time_s', 'pos_x_m', 'pos_y_m', 'pos_z_m', ...
                     'pitch_rad', ...
                     'cg_body_x_m', 'cg_body_y_m', 'cg_body_z_m', ...
                     'nozzle_angle_rad'});

% Write to CSV
writetable(trajectory_table, fullfile(projectRoot, 'Simulator_Core', 'output_data', 'trajectory_for_blender_100fps.csv'));
fprintf('\nExported trajectory data to Simulator_Core/output_data/trajectory_for_blender_100fps.csv\n');

% --- END: Updated Data Export Section ---


