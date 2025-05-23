clear
clc
close all



%% Plant model (TVC Rocket - No aero)
%Define Constants
% Physical parameters (SI units)
m = 1000;             % rocket mass (kg)
rho = 0.3;            % air density (kg/m³) - varies with altitude
V = 400;              % velocity (m/s)
S = 0.5;              % reference area (m²)
L = 9.5;              % reference length (m)
I = 5000;             % moment of inertia about pitch axis (kg⋅m²)
T = 50000;            % thrust (N)
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




%% State Space Representation
A = [-d1 0 1;
     0 0 1;
     d3 0 d4];

B = [-d5;
     0;
     d2];

C = eye(3);    % 3x3 identity - output α, θ, θ̇ directly
D = zeros(3,1); % No direct feedthrough


open_loop_poles = eig(A)



%% Design Full State Feedback Controller
%Verify that the system is controllable
Pc = ctrb(A,B)
rank(Pc)



closed_loop_pole_locations = [-1 -1.1 -1.2]

K_manual_placement = place(A, B, closed_loop_pole_locations)

%% Simulate System
t_final = 20;

x0 = [-5*pi/180;    % α₀ = 5 degrees initial AoA disturbance
      0*pi/180;           % θ₀ = 0 degrees initial pitch angle  
      0];          % θ̇₀ = 0 rad/s initial pitch rate

simOut = sim('TVC_FSFB_model.slx');
logsout = simOut.logsout;

% Extract simulation data
sim_X = logsout.getElement('x').Values;

t = sim_X.Time;
x1 = sim_X.Data(:,1);  % alpha (AoA)
x2 = sim_X.Data(:,2);  % theta (pitch angle)
x3 = sim_X.Data(:,3);  % theta_dot (pitch rate)

sim_Va = logsout.getElement('NozzleAngle').Values;
NozzleAngle = sim_Va.Data(:,1);

%% Create Plots
figure('Position', [100 100 800 600]);

% State plots
subplot(4,1,1)
plot(t, x1*180/pi, 'LineWidth', 2)
ylabel('α (deg)')
title('State Variables')
grid on

subplot(4,1,2)
plot(t, x2*180/pi, 'LineWidth', 2)
ylabel('θ (deg)')
grid on

subplot(4,1,3)
plot(t, x3*180/pi, 'LineWidth', 2)
ylabel('θ̇ (deg/s)')
grid on

subplot(4,1,4)
plot(t, NozzleAngle*180/pi, 'LineWidth', 2)
ylabel('δT (deg)')
xlabel('Time (s)')
title('Control Signal')
grid on

% Alternative: All states on one plot
figure('Position', [200 200 800 400]);
plot(t, x1*180/pi, 'LineWidth', 2, 'DisplayName', 'α (deg)');
hold on
plot(t, x2*180/pi, 'LineWidth', 2, 'DisplayName', 'θ (deg)');
plot(t, x3*180/pi, 'LineWidth', 2, 'DisplayName', 'θ̇ (deg/s)');
plot(t, NozzleAngle*180/pi, '--', 'LineWidth', 2, 'DisplayName', 'δT (deg)');
hold off
xlabel('Time (s)')
ylabel('Angle (deg) / Rate (deg/s)')
title('TVC Rocket Response with Full State Feedback')
legend('Location', 'best')
grid on
