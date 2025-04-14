% STEVE Rocket Simulation Results Visualizer
% This script visualizes results from a previously run simulation
% without re-running the entire simulation. It loads the saved
% simulation data and displays it in an interactive multi-tabbed interface.

clear; clc; close all;

% Prompt user to select results file if not specified
dataPath = '../../data/results/RocketSimData.mat';
if ~exist(dataPath, 'file')
    [file, path] = uigetfile('*.mat', 'Select Rocket Simulation Results File');
    if file == 0
        error('No simulation results file selected. Cannot continue.');
    end
    dataPath = fullfile(path, file);
end

% Load simulation results
fprintf('Loading simulation results from %s\n', dataPath);
load(dataPath);

% Check if required variables are available
requiredVars = {'t', 'Ve', 'Xe', 'Vb', 'phi', 'theta', 'psi', 'alpha', ...
                'Fwind', 'Mwind', 'Fthrust', 'Mthrust', 'Fnet', 'Mnet', ...
                'w', 'nozzleAngle', 'windVelocity'};
                
missingVars = [];
for i = 1:length(requiredVars)
    if ~exist(requiredVars{i}, 'var')
        missingVars{end+1} = requiredVars{i};
    end
end

if ~isempty(missingVars)
    warning('Some variables are missing from the results file: %s', strjoin(missingVars, ', '));
    fprintf('Attempting to continue with available data...\n');
end

% Convert earth frame z to make "up" positive
if exist('Xe', 'var') && ~isempty(Xe)
    Xe(:,3) = -Xe(:,3);  % Invert z-position (altitude)
end
if exist('Ve', 'var') && ~isempty(Ve)
    Ve(:,3) = -Ve(:,3);  % Invert z-velocity
end

% Create main figure with tabs
fig = figure('Name', 'Rocket Simulation Results', 'Position', [100, 100, 1200, 700]);
tabgp = uitabgroup(fig);

% Tab 1: Position
tab1 = uitab(tabgp, 'Title', 'Position');

if exist('Xe', 'var') && ~isempty(Xe)
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
else
    annotation(tab1, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Position data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Tab 2: Velocity
tab2 = uitab(tabgp, 'Title', 'Velocity');

if exist('Ve', 'var') && exist('Vb', 'var') && ~isempty(Ve) && ~isempty(Vb)
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
else
    annotation(tab2, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Velocity data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Tab 3: Attitude and Angular Rates
tab3 = uitab(tabgp, 'Title', 'Attitude');

if exist('theta', 'var') && exist('alpha', 'var') && exist('w', 'var') && exist('nozzleAngle', 'var') && ...
   ~isempty(theta) && ~isempty(alpha) && ~isempty(w) && ~isempty(nozzleAngle)
    plot_height = 0.25;
    
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
else
    annotation(tab3, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Attitude data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Calculate and display key flight metrics
tab4 = uitab(tabgp, 'Title', 'Flight Metrics');

% Only calculate metrics if we have the required data
if exist('Xe', 'var') && exist('Ve', 'var') && exist('alpha', 'var') && ...
   ~isempty(Xe) && ~isempty(Ve) && ~isempty(alpha)
    % Calculate metrics
    max_altitude = max(Xe(:,3));
    [max_speed, max_speed_idx] = max(sqrt(sum(Ve.^2, 2)));
    horiz_dist = sqrt(Xe(end,1)^2 + Xe(end,2)^2);
    max_horiz_dist = max(sqrt(sum(Xe(:,1:2).^2, 2)));
    flight_time = t(end);
    [max_accel, max_accel_idx] = max(diff(sqrt(sum(Ve.^2, 2)))./diff(t));
    max_aoa = max(abs(alpha)) * 180/pi;
    
    % Check if we have engine burn time information
    if exist('Actuators', 'var') && isfield(Actuators, 'Engine') && isfield(Actuators.Engine, 'BurnTime')
        burnout_time = Actuators.Engine.BurnTime;
        % Find the index of the time closest to burnout time
        [~, burnout_idx] = min(abs(t - burnout_time));
        burnout_altitude = Xe(burnout_idx,3);
        burnout_speed = sqrt(sum(Ve(burnout_idx,:).^2));
        burnout_horiz_dist = sqrt(Xe(burnout_idx,1)^2 + Xe(burnout_idx,2)^2);
    else
        burnout_time = NaN;
        burnout_altitude = NaN;
        burnout_speed = NaN;
        burnout_horiz_dist = NaN;
    end
    
    % Create text display for metrics
    metrics_text = {
        sprintf('Maximum Altitude: %.1f km', max_altitude/1000.0)
        sprintf('Maximum Speed: %.1f m/s (at t=%.1f s)', max_speed, t(max_speed_idx))
        sprintf('Final Distance from Launch Pad: %.1f km', horiz_dist/1000.0)
        sprintf('Maximum Distance from Launch Pad: %.1f km', max_horiz_dist/1000.0)
        sprintf('Total Flight Time: %.1f s', flight_time)
        sprintf('Maximum Acceleration: %.1f m/s²', max_accel)
        sprintf('Maximum Angle of Attack: %.1f degrees', max_aoa)
    };
    
    if ~isnan(burnout_time)
        burnout_metrics = {
            sprintf('Burnout Time: %.1f s', burnout_time)
            sprintf('Burnout Altitude: %.1f m', burnout_altitude)
            sprintf('Burnout Speed: %.1f m/s', burnout_speed)
            sprintf('Burnout Horizontal Distance: %.1f m', burnout_horiz_dist)
        };
        metrics_text = [metrics_text; burnout_metrics];
    end
    
    % Display metrics in a clean text box
    annotation(tab4, 'textbox', [0.1 0.1 0.8 0.8], ...
        'String', metrics_text, ...
        'FontSize', 14, ...
        'FontName', 'Consolas', ...
        'EdgeColor', 'none', ...
        'VerticalAlignment', 'middle');
    
    % Print metrics to command window as well
    fprintf('\n=== Key Flight Metrics ===\n');
    fprintf('%s\n', metrics_text{:});
else
    annotation(tab4, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Flight metrics data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Tab 5: Forces
tab5 = uitab(tabgp, 'Title', 'Forces');

if exist('Fwind', 'var') && exist('Fthrust', 'var') && exist('Fnet', 'var') && ...
   ~isempty(Fwind) && ~isempty(Fthrust) && ~isempty(Fnet)
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
else
    annotation(tab5, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Force data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Tab 6: Moments
tab6 = uitab(tabgp, 'Title', 'Moments');

if exist('Mwind', 'var') && exist('Mthrust', 'var') && exist('Mnet', 'var') && ...
   ~isempty(Mwind) && ~isempty(Mthrust) && ~isempty(Mnet)
    % Pitching Moments (My)
    axes('Parent', tab6, 'Position', [0.1, 0.55, 0.8, 0.35]);
    plot(t, Mwind(:,2), 'b-', 'LineWidth', 2); hold on;
    plot(t, Mthrust(:,2), 'r-', 'LineWidth', 2);
    plot(t, Mnet(:,2), 'k-', 'LineWidth', 2);
    grid on; xlabel('Time (s)'); ylabel('Moment (N·m)');
    title('Pitching Moments (M_y)');
    legend('Wind', 'Thrust', 'Net', 'Location', 'best');
else
    annotation(tab6, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Moment data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

% Tab 7: Wind Velocity
tab7 = uitab(tabgp, 'Title', 'Wind Velocity');

if exist('windVelocity', 'var') && ~isempty(windVelocity)
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
else
    annotation(tab7, 'textbox', [0.1, 0.1, 0.8, 0.8], 'String', 'Wind data not available', ...
        'FontSize', 14, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');
end

fprintf('\nResults visualization complete!\n');
