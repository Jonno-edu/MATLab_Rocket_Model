clear
clc
close all

%% Plant model (TVC Rocket - with Actuator Model)
%Define Constants
% Physical parameters (SI units)
m = 974;             % rocket mass (kg)
rho = 0.038;            % air density (kg/m³) - varies with altitude
V = 392;              % velocity (m/s)
S = 0.200296;              % reference area (m²)
L = 9.542;              % reference length (m)
I = 19000;             % moment of inertia about pitch axis (kg⋅m²)
T = 27.6*10^3;            % thrust (N)
L_arm = 4;          % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients (per radian)
CLa = 2.5;            % lift curve slope (1/rad)
Cma = 2;           % moment curve slope (1/rad)
Cmq = -2.0;           % pitch damping coefficient (1/rad)

% Dynamic pressure
q = 0.5 * rho * V^2;  % dynamic pressure (Pa or N/m²)

% Calculate d constants (SI units)
d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);      % [1/s]
d2 = T * L_arm / I;                                 % [rad/s² per rad]
d3 = (rho * V^2 * S * L)/(2 * I) * Cma;           % [rad/s² per rad]  
d4 = (rho * V * S * L^2)/(2 * I) * Cmq;           % [1/s]
d5 = T / (m * V);                                   % [1/s]

% Display results with units
fprintf('d1 = %.4f (1/s) - AoA aerodynamic + thrust normal effect\n', d1);
fprintf('d2 = %.4f (rad/s² per rad) - TVC moment authority\n', d2);  
fprintf('d3 = %.4f (rad/s² per rad) - Static stability\n', d3);
fprintf('d4 = %.4f (1/s) - Pitch damping\n', d4);
fprintf('d5 = %.4f (1/s) - Thrust normal force coefficient\n', d5);

%% Actuator Model Parameters
% Typical TVC gimbal actuator (hydraulic/electric servo)
omega_n_act = 20;        % Natural frequency (rad/s) - 20 rad/s ≈ 3 Hz bandwidth
zeta_act = 0.7;          % Damping ratio (well-damped)
actuator_gain = 1;       % Steady-state gain

fprintf('\nActuator Parameters:\n');
fprintf('Natural frequency: %.1f rad/s (%.1f Hz)\n', omega_n_act, omega_n_act/(2*pi));
fprintf('Damping ratio: %.2f\n', zeta_act);

%% Augmented State Space (Plant + Actuator)
% States: [alpha, theta, theta_dot, actuator_position, actuator_velocity]

% Original plant A matrix (3x3)
A_plant = [-d1 0 1;
           0 0 1;
           d3 0 d4];

% Plant coupling to actuator position (replaces original B matrix)
B_plant_to_actuator = [-d5; 0; d2];  % Effect of actual nozzle deflection

% Actuator dynamics (2x2)
A_actuator = [0, 1;
              -omega_n_act^2, -2*zeta_act*omega_n_act];

B_actuator = [0; omega_n_act^2 * actuator_gain];

% Augmented system matrices (5x5)
A_augmented = [A_plant,                 B_plant_to_actuator, zeros(3,1);
               zeros(2,3),              A_actuator];

B_augmented = [zeros(3,1);
               B_actuator];

% Output matrices - output all states for analysis
C_augmented = eye(5);    % Output all 5 states
D_augmented = zeros(5,1); % No direct feedthrough

% Check system properties
open_loop_poles_augmented = eig(A_augmented);
fprintf('\nAugmented system open-loop poles:\n');
disp(open_loop_poles_augmented)

%% Verify Controllability
Pc_augmented = ctrb(A_augmented, B_augmented);
rank_augmented = rank(Pc_augmented);
fprintf('Augmented system controllability rank: %d (should be 5)\n', rank_augmented);

%% Design LQR Controller for Augmented System

% Bryson's Rule for augmented system
Q_bryson_aug = diag([1/(5*pi/180)^2,    % Max 5° AoA acceptable
                     1/(5*pi/180)^2,    % Max 5° pitch acceptable  
                     1/(50*pi/180)^2,   % Max 50°/s pitch rate
                     1/(4*pi/180)^2,    % Max 4° actuator position
                     1/(50*pi/180)^2]); % Max 50°/s actuator rate

R_bryson_aug = 1/(4*pi/180)^2;          % Max 4° nozzle command

% Use Bryson's rule
Q_augmented = Q_bryson_aug;
R_augmented = R_bryson_aug;

% Calculate LQR gain for augmented system
K_lqr_augmented = lqr(A_augmented, B_augmented, Q_augmented, R_augmented);
fprintf('\nLQR gains (augmented): K = [%.4f %.4f %.4f %.4f %.4f]\n', K_lqr_augmented);

% Check closed-loop poles
A_cl_augmented = A_augmented - B_augmented*K_lqr_augmented;
closed_loop_poles_augmented = eig(A_cl_augmented);
fprintf('Augmented system closed-loop poles:\n');
disp(closed_loop_poles_augmented)

%% Simulate System
t_final = 10;

% Initial conditions: [alpha, theta, theta_dot, act_pos, act_vel]
x0_augmented = [5*pi/180;    % α₀ = 5 degrees initial AoA disturbance
                0;           % θ₀ = 0 degrees initial pitch angle  
                0;           % θ̇₀ = 0 rad/s initial pitch rate
                0;           % actuator initial position
                0];          % actuator initial velocity

% Create closed-loop system
sys_cl_augmented = ss(A_cl_augmented, B_augmented, C_augmented, D_augmented);

% Simulate step response with initial conditions
t = 0:0.01:t_final;
u_command = zeros(size(t));  % No external command (just initial condition response)
[y_augmented, t_sim] = lsim(sys_cl_augmented, u_command, t, x0_augmented);

% Extract states
x1_aug = y_augmented(:,1);  % alpha (AoA)
x2_aug = y_augmented(:,2);  % theta (pitch angle)
x3_aug = y_augmented(:,3);  % theta_dot (pitch rate)
x4_aug = y_augmented(:,4);  % actuator position (actual nozzle angle)
x5_aug = y_augmented(:,5);  % actuator velocity

% Calculate control command
u_command_sim = -K_lqr_augmented * y_augmented';

%% Create Plots
figure('Position', [100 100 1000 800]);

% Plant state plots
subplot(3,2,1)
plot(t_sim, x1_aug*180/pi, 'LineWidth', 2)
ylabel('α (deg)')
title('Angle of Attack')
grid on

subplot(3,2,2)
plot(t_sim, x2_aug*180/pi, 'LineWidth', 2)
ylabel('θ (deg)')
title('Pitch Angle')
grid on

subplot(3,2,3)
plot(t_sim, x3_aug*180/pi, 'LineWidth', 2)
ylabel('θ̇ (deg/s)')
title('Pitch Rate')
grid on

% Actuator plots
subplot(3,2,4)
plot(t_sim, x4_aug*180/pi, 'LineWidth', 2)
ylabel('δT (deg)')
title('Actual Nozzle Deflection')
grid on

subplot(3,2,5)
plot(t_sim, x5_aug*180/pi, 'LineWidth', 2)
ylabel('δ̇T (deg/s)')
title('Nozzle Deflection Rate')
grid on

subplot(3,2,6)
plot(t_sim, u_command_sim*180/pi, 'LineWidth', 2)
ylabel('δT_cmd (deg)')
title('Commanded Nozzle Deflection')
xlabel('Time (s)')
grid on

% Combined plot showing command vs actual
figure('Position', [200 200 800 600]);
plot(t_sim, u_command_sim*180/pi, '--', 'LineWidth', 2, 'DisplayName', 'Commanded δT');
hold on
plot(t_sim, x4_aug*180/pi, 'LineWidth', 2, 'DisplayName', 'Actual δT');
plot(t_sim, x1_aug*180/pi, 'LineWidth', 2, 'DisplayName', 'α (AoA)');
hold off
xlabel('Time (s)')
ylabel('Angle (deg)')
title('TVC Rocket Response with Actuator Dynamics')
legend('Location', 'best')
grid on

% Performance summary
fprintf('\nPerformance Summary:\n');
fprintf('Max control command: %.2f degrees\n', max(abs(u_command_sim))*180/pi);
fprintf('Max actual deflection: %.2f degrees\n', max(abs(x4_aug))*180/pi);
fprintf('AoA settling time: %.2f seconds\n', find(abs(x1_aug) < 0.02*x0_augmented(1), 1, 'first') * 0.01);
