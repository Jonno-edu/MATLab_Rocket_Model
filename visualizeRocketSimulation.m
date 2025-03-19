%clc; close all;

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

% Extract force and moment data
Fwind = out.Fwind.Data;
Mwind = out.Mwind.Data;
Fthrust = out.Fthrust.Data;
Mthrust = out.Mthrust.Data;
Fnet = out.Fnet.Data;
Mnet = out.Mnet.Data;

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

% Plot theta (pitch)
axes('Parent', tab3, 'Position', [0.1, 0.7, 0.8, 0.25]);
plot(t, theta*180/pi, 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle (theta)');

% Plot angle of attack
axes('Parent', tab3, 'Position', [0.1, 0.4, 0.8, 0.25]);
plot(t, alpha*180/pi, 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Angle of Attack');

% Plot angular rate (omega)
axes('Parent', tab3, 'Position', [0.1, 0.1, 0.8, 0.25]);
plot(t, w(:,2)*180/pi, 'r-', 'LineWidth', 2);  % Pitch rate (y-axis rotation)
grid on; xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Pitch Rate (omega_y)');


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

% Save trajectory data for Python visualization
trajectory_data = [t, Xe, theta];
writematrix(trajectory_data, 'rocket_trajectory.csv');
fprintf('Trajectory data saved to rocket_trajectory.csv\n');


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
