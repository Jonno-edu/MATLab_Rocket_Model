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

pitch_controller_cmd = rad2deg(logsout.getElement('pitch_controller_cmd').Values.Data);
t_pitch_controller_cmd = logsout.getElement('pitch_controller_cmd').Values.Time;

hold on;
% Plot pitch rate controller command (dotted line)
plot(t_pitch_controller_cmd, pitch_controller_cmd, 'k:');

legend('Actual q', 'Controller q_{cmd}');

plot(t_w, q, 'b');
title('Pitch Rate');
ylabel('q (deg/s)');

% % 4. Nozzle commands and actual angle
% subplot(4,1,4);
% hold on;
% grid on;
% 
% % Get commands and actual angle
% ff_cmd = rad2deg(logsout.getElement('feedforward_cmd').Values.Data);
% t_ff = logsout.getElement('feedforward_cmd').Values.Time;
% 
% ctrl_cmd = rad2deg(logsout.getElement('controller_cmd').Values.Data);
% t_ctrl = logsout.getElement('controller_cmd').Values.Time;
% 
% total_cmd = rad2deg(logsout.getElement('total_nozzle_angle_rad').Values.Data);
% t_total = logsout.getElement('total_nozzle_angle_rad').Values.Time;
% 
% nozzle_angle = rad2deg(logsout.getElement('Y_nozzle_angle').Values.Data);
% t_nozzle = logsout.getElement('Y_nozzle_angle').Values.Time;
% 
% plot(t_ff, ff_cmd, 'g--');
% plot(t_ctrl, ctrl_cmd, 'r--');
% plot(t_total, total_cmd, 'm--');
% plot(t_nozzle, nozzle_angle, 'b');
% 
% title('Nozzle Angles');
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% legend('Feedforward', 'Controller', 'Total', 'Actual');
% end
