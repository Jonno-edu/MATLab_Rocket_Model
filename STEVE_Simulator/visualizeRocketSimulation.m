clc; close all;

initialize_parameters;

mdl = "STEVE_Simulation";
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

% Skip the first N frames
N = 2;  % Number of frames to skip
t = t(N+1:end);
Ve = Ve(N+1:end,:);
Xe = Xe(N+1:end,:);
Vb = Vb(N+1:end,:);
phi = phi(N+1:end);
theta = theta(N+1:end);
psi = psi(N+1:end);
alpha = alpha(N+1:end);


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
% No need for YDir 'reverse' since we modified the data

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
% No need for YDir 'reverse' since we modified the data

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

% Tab 3: Attitude (theta only)
tab3 = uitab(tabgp, 'Title', 'Attitude');

% Only plot theta (pitch) since that's the only active rotation
axes('Parent', tab3, 'Position', [0.1, 0.55, 0.8, 0.35]);
plot(t, theta*180/pi, 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle (theta)');

% Angle of attack
axes('Parent', tab3, 'Position', [0.1, 0.1, 0.8, 0.35]);
plot(t, alpha*180/pi, 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Angle of Attack');

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

% Save trajectory data for Python visualization
trajectory_data = [t, Xe, theta];
writematrix(trajectory_data, 'rocket_trajectory.csv');
fprintf('Trajectory data saved to rocket_trajectory.csv\n');
