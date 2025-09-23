function plot_simulation_results(simOut)
% Simple function to plot rocket attitude and control data

%% 1. Setup
logsout = simOut.logsout;
downsample_factor = 20; % Downsampling factor for faster plotting

% Extract the main data buses once to simplify access
plant_data = logsout.getElement('PlantData').Values;
cmd_data = logsout.getElement('CMD').Values;

%% 2. Plot Attitude and Control
figure('Name', 'Attitude and Control');

% --- Theta angle and command ---
subplot(4,1,1);
hold on; grid on;

% Get actual theta
theta_ts = plant_data.body_angles.theta;
theta = rad2deg(theta_ts.Data);
t_theta = theta_ts.Time;
idx_theta = 1:downsample_factor:length(theta);
plot(t_theta(idx_theta), theta(idx_theta), 'b', 'DisplayName', 'Actual \theta');

% Get theta command
theta_cmd_ts = cmd_data.ThetaCmd;
% Corrected .data to .Data
theta_cmd = rad2deg(theta_cmd_ts.Data); 
t_cmd = theta_cmd_ts.Time;
idx_cmd = 1:downsample_factor:length(theta_cmd);
plot(t_cmd(idx_cmd), theta_cmd(idx_cmd), 'r--', 'DisplayName', 'Command \theta');

title('Pitch Angle and Command');
legend('show');
ylabel('Angle (deg)');

% --- Angle of attack ---
subplot(4,1,2);
hold on; grid on;

alpha_ts = plant_data.alpha;
alpha = rad2deg(alpha_ts.Data);
if length(alpha) >= 2, alpha(1:2) = 0; end % Set first two values to zero
t_alpha = alpha_ts.Time;
idx_alpha = 1:downsample_factor:length(alpha);
plot(t_alpha(idx_alpha), alpha(idx_alpha), 'b');
plot([t_alpha(1) t_alpha(end)], [5 5], 'r--');
plot([t_alpha(1) t_alpha(end)], [-5 -5], 'r--');
title('Angle of Attack');
ylabel('\alpha (deg)');

% --- Pitch rate ---
subplot(4,1,3);
grid on;

% Corrected to get q from the 'w' bus structure
q_ts = plant_data.w.theta_dot; 
q = rad2deg(q_ts.Data);
t_q = q_ts.Time;
idx_q = 1:downsample_factor:length(q);
plot(t_q(idx_q), q(idx_q), 'b', 'DisplayName', 'Actual q');
title('Pitch Rate');
ylabel('q (deg/s)');
legend('show');

% --- Nozzle commands and actual angle ---
subplot(4,1,4);
hold on; grid on;

total_cmd_ts = logsout.getElement('<nozzle_angle_cmd>').Values;
total_cmd = rad2deg(total_cmd_ts.Data);
t_total = total_cmd_ts.Time;
idx_total = 1:downsample_factor:length(total_cmd);
plot(t_total(idx_total), total_cmd(idx_total), 'm--', 'DisplayName', 'Command');

nozzle_angle_ts = logsout.getElement('Y_nozzle_angle').Values;
nozzle_angle = rad2deg(nozzle_angle_ts.Data);
t_nozzle = nozzle_angle_ts.Time;
idx_nozzle = 1:downsample_factor:length(nozzle_angle);
plot(t_nozzle(idx_nozzle), nozzle_angle(idx_nozzle), 'b', 'DisplayName', 'Actual');

title('Nozzle Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('show');

%% 3. Plot Body Forces and Moments
figure('Name', 'Body Forces and Y-Axis Moment');

% Get force and moment components
F_total_ts   = logsout.getElement('F_total').Values;
F_aero_ts    = logsout.getElement('F_aero').Values;
F_thrust_ts  = logsout.getElement('F_thrust').Values;
% Corrected to use ENV bus for gravity
F_gravity_ts = logsout.getElement('ENV').Values.Fg; 
M_total_ts   = logsout.getElement('M_total').Values;
M_aero_ts    = logsout.getElement('M_aero').Values;
M_thrust_ts  = logsout.getElement('M_thrust').Values;

t_forces = F_total_ts.Time;
t_moments = M_total_ts.Time;

idx_forces = 1:downsample_factor:length(t_forces);
t_forces_ds = t_forces(idx_forces);
idx_moments = 1:downsample_factor:length(t_moments);
t_moments_ds = t_moments(idx_moments);

% Plot X-axis force (vertical)
subplot(3,1,1);
hold on; grid on;
plot(t_forces_ds, F_total_ts.Data(idx_forces,1),   'k',  'LineWidth', 1.2, 'DisplayName', 'Total');
plot(t_forces_ds, F_aero_ts.Data(idx_forces,1),    'b--', 'LineWidth', 1.2, 'DisplayName', 'Aero');
plot(t_forces_ds, F_thrust_ts.Data(idx_forces,1),  'r--', 'LineWidth', 1.2, 'DisplayName', 'Thrust');
plot(t_forces_ds, F_gravity_ts.Data(idx_forces,1),'g--', 'LineWidth', 1.2, 'DisplayName', 'Gravity');
title('Body X-axis Force (Vertical)');
ylabel('F_x (N)');
legend('show');

% Plot Z-axis force
subplot(3,1,2);
hold on; grid on;
plot(t_forces_ds, F_total_ts.Data(idx_forces,3),   'k',  'LineWidth', 1.2, 'DisplayName', 'Total');
plot(t_forces_ds, F_aero_ts.Data(idx_forces,3),    'b--', 'LineWidth', 1.2, 'DisplayName', 'Aero');
plot(t_forces_ds, F_thrust_ts.Data(idx_forces,3),  'r--', 'LineWidth', 1.2, 'DisplayName', 'Thrust');
plot(t_forces_ds, F_gravity_ts.Data(idx_forces,3),'g--', 'LineWidth', 1.2, 'DisplayName', 'Gravity');
title('Body Z-axis Force');
ylabel('F_z (N)');
legend('show');

% Plot Y-axis moment
subplot(3,1,3);
hold on; grid on;
plot(t_moments_ds, M_total_ts.Data(idx_moments,2),   'k',  'LineWidth', 1.2, 'DisplayName', 'Total');
plot(t_moments_ds, M_aero_ts.Data(idx_moments,2),    'b--', 'LineWidth', 1.2, 'DisplayName', 'Aero');
plot(t_moments_ds, M_thrust_ts.Data(idx_moments,2),  'r--', 'LineWidth', 1.2, 'DisplayName', 'Thrust');
title('Body Y-axis Moment');
ylabel('M_y (N-m)');
xlabel('Time (s)');
legend('show');

%% 4. Plot Aerodynamic Forces (Wind Axis)
figure('Name', 'Aerodynamic Forces in Wind Axis');

F_aero_data = F_aero_ts.Data(idx_forces, :);
t_aero_ds = t_forces_ds;

% Initialize array for wind-axis forces
F_aero_wind = zeros(size(F_aero_data));

% Get DCM data for transformation
DCMwb_ts = plant_data.DCMwb;
DCMwb_data = DCMwb_ts.Data;
t_DCM = DCMwb_ts.Time;

% Transform each force vector from body to wind frame
for i = 1:size(F_aero_data, 1)
    idx_dcm = find(t_DCM >= t_aero_ds(i), 1, 'first');
    if isempty(idx_dcm), idx_dcm = length(t_DCM); end
    DCM_i = squeeze(DCMwb_data(:,:,idx_dcm));
    F_aero_wind(i,:) = (DCM_i * F_aero_data(i,:)')';
end

% Plot drag force (-X in wind frame)
subplot(2,1,1);
hold on; grid on;
plot(t_aero_ds, -F_aero_wind(:,1), 'b', 'LineWidth', 1.2);
title('Drag Force');
ylabel('Drag (N)');

% Plot normal force (-Z in wind frame)
subplot(2,1,2);
hold on; grid on;
plot(t_aero_ds, -F_aero_wind(:,3), 'r', 'LineWidth', 1.2);
title('Normal Force');
ylabel('Normal Force (N)');
xlabel('Time (s)');

end
