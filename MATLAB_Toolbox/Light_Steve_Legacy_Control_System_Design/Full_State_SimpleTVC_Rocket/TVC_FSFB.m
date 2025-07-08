clear
clc
close all

%% Plant model (TVC Rocket - with Actuator Model)
% Physical parameters (SI units)
m = 974;             % rocket mass (kg)
rho = 0.038;         % air density (kg/m³)
V = 392;             % velocity (m/s)
S = 0.200296;        % reference area (m²)
L = 9.542;           % reference length (m)
I = 19000;           % moment of inertia (kg⋅m²)
T = 27.6*10^3;       % thrust (N)
L_arm = 4;           % TVC moment arm (m)

% Aerodynamic coefficients (per radian)
CLa = 2.5;           % lift curve slope (1/rad)
Cma = 2;             % moment curve slope (1/rad)
Cmq = -2.0;          % pitch damping coefficient (1/rad)

% Dynamic pressure
q = 0.5 * rho * V^2;

% Calculate d constants
d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);
d2 = T * L_arm / I;
d3 = (rho * V^2 * S * L)/(2 * I) * Cma;
d4 = (rho * V * S * L^2)/(2 * I) * Cmq;
d5 = T / (m * V);

fprintf('d1 = %.4f (1/s)\n', d1);
fprintf('d2 = %.4f (rad/s² per rad)\n', d2);
fprintf('d3 = %.4f (rad/s² per rad)\n', d3);
fprintf('d4 = %.4f (1/s)\n', d4);
fprintf('d5 = %.4f (1/s)\n', d5);

%% Actuator Model
omega_n_act = 20;
zeta_act = 0.7;
actuator_gain = 1;

%% Augmented State Space
A_plant = [-d1 0 1;
           0 0 1;
           d3 0 d4];

B_plant_to_actuator = [-d5; 0; d2];

A_actuator = [0, 1;
              -omega_n_act^2, -2*zeta_act*omega_n_act];

B_actuator = [0; omega_n_act^2 * actuator_gain];

A_augmented = [A_plant, B_plant_to_actuator, zeros(3,1);
               zeros(2,3), A_actuator];

B_augmented = [zeros(3,1); B_actuator];

C_augmented = eye(5);
D_augmented = zeros(5,1);

% Check open-loop poles
open_loop_poles = eig(A_augmented);
fprintf('\nOpen-loop poles:\n');
disp(open_loop_poles)

%% LQR Design
Q_augmented = diag([1/(15*pi/180)^2,    % alpha
                    1/(5*pi/180)^2,    % theta
                    1/(50*pi/180)^2,   % theta_dot
                    1/(4*pi/180)^2,    % actuator position
                    1/(50*pi/180)^2]); % actuator velocity

R_augmented = 1/(4*pi/180)^2;

K_lqr_augmented = lqr(A_augmented, B_augmented, Q_augmented, R_augmented);
fprintf('\nLQR gains: K = [%.4f %.4f %.4f %.4f %.4f]\n', K_lqr_augmented);

% Check closed-loop poles
A_cl = A_augmented - B_augmented*K_lqr_augmented;
closed_loop_poles = eig(A_cl);
fprintf('\nClosed-loop poles:\n');
disp(closed_loop_poles)

%% Step Response Implementation
t_final = 60;
t = 0:0.01:t_final;

% Zero initial conditions
x0_augmented = zeros(5,1);

% Create 5° step command for theta starting at t=1s
theta_ref = zeros(size(t));
theta_ref(t >= 1) = 0.5*pi/180;  % 5° step in radians

% For reference tracking, we need to add reference input to theta equation
% This means the reference affects the theta_dot equation (row 2 of A matrix)
B_ref = [0; 1; 0; 0; 0];  % Reference input affects theta_dot directly

% Calculate feedforward gain for zero steady-state error
% At steady state: 0 = A_cl*x_ss + B_ref*theta_ref
% So: x_ss = -inv(A_cl)*B_ref*theta_ref
% We want theta_ss = theta_ref, so we need the feedforward gain
Nbar = -inv([A_cl, B_ref; [0 1 0 0 0], 0]) * [zeros(5,1); 1];
Nx = Nbar(1:5);  % State feedforward
Nu = Nbar(6);    % Input feedforward

fprintf('\nFeedforward gains:\n');
fprintf('Nx = [%.4f %.4f %.4f %.4f %.4f]\n', Nx);
fprintf('Nu = %.4f\n', Nu);

% Create the complete closed-loop system with reference tracking
% x_dot = A_cl*x + B_ref*theta_ref + B_augmented*Nu*theta_ref
% u = -K*x + Nu*theta_ref
B_total = B_ref + B_augmented*Nu;

sys_tracking = ss(A_cl, B_total, C_augmented, D_augmented);

% Simulate step response
[y_step, t_sim] = lsim(sys_tracking, theta_ref, t, x0_augmented);

% Extract states
x1_step = y_step(:,1);  % alpha (AoA)
x2_step = y_step(:,2);  % theta (pitch angle)
x3_step = y_step(:,3);  % theta_dot (pitch rate)
x4_step = y_step(:,4);  % actuator position
x5_step = y_step(:,5);  % actuator velocity

% Calculate control commands
u_step = -K_lqr_augmented * y_step' + Nu * theta_ref;

%% Performance Analysis
% Calculate tracking error
tracking_error = theta_ref' - x2_step;

% Find settling time (2% criterion)
final_value = theta_ref(end);
settling_threshold = 0.02 * final_value;
settling_indices = find(abs(tracking_error) < settling_threshold);
if ~isempty(settling_indices)
    settling_time = t_sim(settling_indices(1));
else
    settling_time = inf;
end

% Find overshoot
max_theta = max(x2_step);
overshoot_percent = (max_theta - final_value) / final_value * 100;

% Steady-state error
ss_error = abs(tracking_error(end));

%% Plotting
figure('Position', [200 200 1000 700]);

% States plot
subplot(2,2,1)
plot(t_sim, theta_ref*180/pi, '--k', 'LineWidth', 2, 'DisplayName', 'θ_{ref}');
hold on
plot(t_sim, x2_step*180/pi, 'r', 'LineWidth', 2, 'DisplayName', 'θ (Pitch)');
plot(t_sim, x1_step*180/pi, 'b', 'LineWidth', 2, 'DisplayName', 'α (AoA)');
ylabel('Angle (deg)')
title('Step Response - Angular States')
legend('Location', 'southeast')
grid on
xlim([0 t_final])

% Control signals
subplot(2,2,2)
plot(t_sim, u_step*180/pi, 'g', 'LineWidth', 2, 'DisplayName', 'Command');
hold on
plot(t_sim, x4_step*180/pi, 'm', 'LineWidth', 2, 'DisplayName', 'Actual δT');
ylabel('Nozzle Angle (deg)')
title('Actuator Response')
legend('Location', 'southeast')
grid on
xlim([0 t_final])

% Tracking error
subplot(2,2,3)
plot(t_sim, tracking_error*180/pi, 'r', 'LineWidth', 2);
ylabel('Tracking Error (deg)')
xlabel('Time (s)')
title('Pitch Angle Tracking Error')
grid on
xlim([0 t_final])

% Pitch rate
subplot(2,2,4)
plot(t_sim, x3_step*180/pi, 'LineWidth', 2);
ylabel('Pitch Rate (deg/s)')
xlabel('Time (s)')
title('Pitch Rate Response')
grid on
xlim([0 t_final])

%% Performance Summary
fprintf('\n=== STEP RESPONSE PERFORMANCE ===\n');
fprintf('Step command: %.1f degrees at t = 1s\n', 5);
fprintf('Settling time (2%%): %.2f seconds\n', settling_time);
fprintf('Overshoot: %.2f%%\n', overshoot_percent);
fprintf('Steady-state error: %.4f degrees\n', ss_error*180/pi);
fprintf('Max control command: %.2f degrees\n', max(abs(u_step))*180/pi);
fprintf('Max actuator deflection: %.2f degrees\n', max(abs(x4_step))*180/pi);
fprintf('Final theta value: %.3f degrees\n', x2_step(end)*180/pi);
