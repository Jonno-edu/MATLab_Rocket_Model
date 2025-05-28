function plot_simulation_results(simOut)
% Simple function to plot rocket attitude and control data
logsout = simOut.logsout;

% Create figure
figure;

% 1. Theta angle and command
subplot(4,1,1);
hold on;
grid on;

% Get actual theta
plant_data = logsout.getElement('PlantData').Values;
theta = rad2deg(plant_data.body_angles.theta.Data);
t_theta = plant_data.body_angles.theta.Time;

% Get theta command
cmd_data = logsout.getElement('CMD').Values;
theta_cmd = rad2deg(cmd_data.ThetaCmd.data);
t_cmd = cmd_data.ThetaCmd.Time;

plot(t_theta, theta, 'b');
plot(t_cmd, theta_cmd, 'r--');
title('Pitch Angle and Command');
legend('Actual \theta', 'Command \theta');
ylabel('Angle (deg)');

% 2. Angle of attack with +/- 5 degree markers
subplot(4,1,2);
hold on;
grid on;

alpha = rad2deg(plant_data.alpha.Data);
% Set first two alpha values to zero
if length(alpha) >= 2
    alpha(1:2) = 0;
end
t_alpha = plant_data.alpha.Time;

plot(t_alpha, alpha, 'b');
plot([t_alpha(1) t_alpha(end)], [5 5], 'r--');
plot([t_alpha(1) t_alpha(end)], [-5 -5], 'r--');
title('Angle of Attack');
ylabel('\alpha (deg)');

% 3. Pitch rate
subplot(4,1,3);
grid on;

% Get q (pitch rate) from angular velocity w
w_data = plant_data.w.Data;
t_w = plant_data.w.Time;
q = rad2deg(w_data(:,2)); % Second column is q


plot(t_w, q, 'b');
title('Pitch Rate');
ylabel('q (deg/s)');

legend('Actual q');

% 4. Nozzle commands and actual angle
subplot(4,1,4);
hold on;
grid on;

% Get commands and actual angle
% ff_cmd = rad2deg(logsout.getElement('feedforward_cmd').Values.Data);
% t_ff = logsout.getElement('feedforward_cmd').Values.Time;
% 
% ctrl_cmd = rad2deg(logsout.getElement('pitchrate_controller_cmd').Values.Data);
% t_ctrl = logsout.getElement('pitchrate_controller_cmd').Values.Time;
% 
total_cmd = rad2deg(logsout.getElement('<nozzle_angle_cmd>').Values.Data);
t_total = logsout.getElement('<nozzle_angle_cmd>').Values.Time;

nozzle_angle = rad2deg(logsout.getElement('Y_nozzle_angle').Values.Data);
t_nozzle = logsout.getElement('Y_nozzle_angle').Values.Time;

% plot(t_ff, ff_cmd, 'g--');
% plot(t_ctrl, ctrl_cmd, 'r--');
plot(t_total, total_cmd, 'm--');
plot(t_nozzle, nozzle_angle, 'b');

title('Nozzle Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Command', 'Actual');
% legend('Actual');

% Create a new figure for forces and moments (reorganized)
figure;
set(gcf, 'Name', 'Body Forces and Y-Axis Moment');

% Get force and moment components
F_total   = logsout.getElement('F_total').Values.Data;   % [Fx Fy Fz]
F_aero    = logsout.getElement('F_aero').Values.Data;
F_thrust  = logsout.getElement('F_thrust').Values.Data;
F_gravity = logsout.getElement('<Fg>').Values.Data;
M_total   = logsout.getElement('M_total').Values.Data;   % [Mx My Mz]
M_aero    = logsout.getElement('M_aero').Values.Data;
M_thrust  = logsout.getElement('M_thrust').Values.Data;

t_forces = logsout.getElement('F_total').Values.Time;
t_moments = logsout.getElement('M_total').Values.Time;

% Plot X-axis force (vertical)
subplot(3,1,1);
hold on; grid on;
plot(t_forces, F_total(:,1),   'k',  'LineWidth', 1.2); % Fx total
plot(t_forces, F_aero(:,1),   'b--', 'LineWidth', 1.2); % Fx aero
plot(t_forces, F_thrust(:,1), 'r--', 'LineWidth', 1.2); % Fx thrust
plot(t_forces, F_gravity(:,1), 'g--', 'LineWidth', 1.2); % Fx gravity
title('Body X-axis Force (Vertical)');
ylabel('F_x (N)');
legend('Total', 'Aerodynamic', 'Thrust', 'Gravity');

% Plot Z-axis force
subplot(3,1,2);
hold on; grid on;
plot(t_forces, F_total(:,3),   'k',  'LineWidth', 1.2); % Fz total
plot(t_forces, F_aero(:,3),   'b--', 'LineWidth', 1.2); % Fz aero
plot(t_forces, F_thrust(:,3), 'r--', 'LineWidth', 1.2); % Fz thrust
plot(t_forces, F_gravity(:,3), 'g--', 'LineWidth', 1.2); % Fz gravity
title('Body Z-axis Force');
ylabel('F_z (N)');
legend('Total', 'Aerodynamic', 'Thrust', 'Gravity');

% Plot Y-axis moment (kept as is)
subplot(3,1,3);
hold on; grid on;
plot(t_moments, M_total(:,2),   'k',  'LineWidth', 1.2); % My total
plot(t_moments, M_aero(:,2),   'b--', 'LineWidth', 1.2); % My aero
plot(t_moments, M_thrust(:,2), 'r--', 'LineWidth', 1.2); % My thrust
title('Body Y-axis Moment');
ylabel('M_y (N-m)');
xlabel('Time (s)');
legend('Total', 'Aerodynamic', 'Thrust');

% Create a figure specifically for aerodynamic forces and moments
figure;
set(gcf, 'Name', 'Aerodynamic Forces and Moments');

% Plot X aerodynamic force
subplot(3,1,1);
hold on; grid on;
plot(t_forces, F_aero(:,1), 'b', 'LineWidth', 1.2); % Fx aero
title('Aerodynamic Force (X axis)');
ylabel('F_x Aero (N)');

% Plot Y and Z aerodynamic forces
subplot(3,1,2);
hold on; grid on;
plot(t_forces, F_aero(:,2), 'g', 'LineWidth', 1.2); % Fy aero
plot(t_forces, F_aero(:,3), 'r', 'LineWidth', 1.2); % Fz aero
title('Aerodynamic Forces (Y and Z axes)');
ylabel('Force (N)');
legend('F_y Aero', 'F_z Aero');

% Plot aerodynamic moments
subplot(3,1,3);
hold on; grid on;
plot(t_moments, M_aero(:,1), 'b', 'LineWidth', 1.2); % Mx aero
plot(t_moments, M_aero(:,2), 'r', 'LineWidth', 1.2); % My aero
plot(t_moments, M_aero(:,3), 'g', 'LineWidth', 1.2); % Mz aero
title('Aerodynamic Moments');
ylabel('Moment (N-m)');
xlabel('Time (s)');
legend('M_x Aero', 'M_y Aero', 'M_z Aero');

% Create a figure for aerodynamic forces in wind axis
figure;
set(gcf, 'Name', 'Aerodynamic Forces in Wind Axis');

% Initialize arrays for wind-axis forces
F_aero_wind = zeros(size(F_aero));

% Get DCM data and times (DCM converts vectors from body to wind frame directly)
DCMwb_data = plant_data.DCMwb.Data;
t_DCM = plant_data.DCMwb.Time;

% Transform each force vector from body to wind frame using a proper time match
for i = 1:size(F_aero, 1)
    % Find the first DCM time greater than or equal to the current force time
    idx = find(t_DCM >= t_forces(i), 1, 'first');
    if isempty(idx)
        idx = length(t_DCM);
    end
    
    % Get DCM at this time point (no transpose needed)
    DCM_i = squeeze(DCMwb_data(:,:,idx));
    
    % Convert from body to wind
    F_aero_wind(i,:) = (DCM_i * F_aero(i,:)')';
end

% Plot drag force (negative X-axis force in wind frame)
subplot(2,1,1);
hold on; grid on;
plot(t_forces, -F_aero_wind(:,1), 'b', 'LineWidth', 1.2);
title('Drag Force');
ylabel('Drag (N)');

% Plot normal force (negative Z-axis force in wind frame)
subplot(2,1,2);
hold on; grid on;
plot(t_forces, -F_aero_wind(:,3), 'r', 'LineWidth', 1.2);
title('Normal Force');
ylabel('Normal Force (N)');
xlabel('Time (s)');

figure;
subplot(2,1,1);
hold on; grid on;
Vb_data = plant_data.Vb.Data;
t_Vb = plant_data.Vb.Time;
plot(t_Vb, Vb_data(:,3), 'b', 'LineWidth', 1.2);
title('Body Z Axis Velocity');
ylabel('V_z (m/s)');

subplot(2,1,2);
hold on; grid on;
plot(t_forces, F_aero(:,3), 'r', 'LineWidth', 1.2);
title('Body Z Axis Aerodynamic Force');
xlabel('Time (s)');
ylabel('F_x Aero (N)');

% Create a new figure for angle of attack and aerodynamic moment plots
figure;
subplot(2,1,1);
hold on; grid on;
plot(t_alpha, alpha, 'b', 'LineWidth', 1.2);
title('Angle of Attack');
ylabel('\alpha (deg)');

subplot(2,1,2);
hold on; grid on;
plot(t_moments, M_aero(:,2), 'r', 'LineWidth', 1.2);
title('Aerodynamic Pitching Moment');
xlabel('Time (s)');
ylabel('M_{pitch} (N-m)');

end
