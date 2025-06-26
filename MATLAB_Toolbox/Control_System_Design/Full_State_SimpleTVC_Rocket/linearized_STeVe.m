clear
clc
close all

%% Plant model (TVC Rocket - Max Q @ t=60s)
% Physical parameters (SI units)
m = 1277.9;             % rocket mass (kg)
rho = 0.5;              % air density (kg/m³)
V = 340.8;              % velocity (m/s) - Transonic
S = 0.200296;           % reference area (m²)
L = 9.542;              % reference length (m)
I = 2.38e4;             % moment of inertia about pitch axis (kg⋅m²)
T = 2.64e4;             % thrust (N)
L_arm = 3.9;            % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients
CLa = 2.0;              % lift curve slope (1/rad)
Cma = 0.885;            % moment curve slope (1/rad) - positive = unstable
Cmq = -1.05;            % pitch damping coefficient (1/rad)

% Calculate derived constants
q = 0.5 * rho * V^2;
d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);
d2 = T * L_arm / I;
d3 = (rho * V^2 * S * L)/(2 * I) * Cma;
d4 = (rho * V * S * L^2)/(2 * I) * Cmq;
d5 = T / (m * V);

%% Basic 3-State Plant
% States: [alpha, theta, theta_dot]
A_plant = [-d1  0   1;
            0   0   1;
            d3  0   d4];

B_plant = [-d5;
            0;
            d2];
        
D_plant = zeros(3,1);

fprintf('=== Transfer Function Generation ===\n');

%% 1. Transfer Function for alpha / nozzle_angle
fprintf('\nG_alpha(s) = alpha(s) / delta(s)\n');
C_alpha = [1 0 0];
sys_alpha = ss(A_plant, B_plant, C_alpha, D_plant(1));
G_alpha = tf(sys_alpha);
G_alpha


%% 2. Transfer Function for theta / nozzle_angle
fprintf('\nG_theta(s) = theta(s) / delta(s)\n');
C_theta = [0 1 0];
sys_theta = ss(A_plant, B_plant, C_theta, D_plant(2));
G_theta = tf(sys_theta);
G_theta


%% Transfer Function for theta_dot / nozzle_angle
fprintf('\nG_thetadot(s) = thetadot(s) / delta(s)\n');
C_thetadot = [0 0 1];
sys_thetadot = ss(A_plant, B_plant, C_thetadot, D_plant(3));
G_thetadot = tf(sys_thetadot);
fprintf('Plant Transfer Function (G_thetadot):\n');
G_thetadot


%% Actuator Model
% Second order actuator parameters
natural_frequency = 62; % rad/s
damping_ratio = 0.5858;

% Create the actuator transfer function
% G_actuator(s) = (wn^2) / (s^2 + 2*zeta*wn*s + wn^2)
num_actuator = natural_frequency^2;
den_actuator = [1, 2*damping_ratio*natural_frequency, natural_frequency^2];
G_actuator = tf(num_actuator, den_actuator);
fprintf('\nActuator Transfer Function (G_actuator):\n');
G_actuator


%% Combined Plant and Actuator Model
% Multiply the transfer functions since they are in series
G_combined = G_thetadot * G_actuator;
fprintf('\nCombined Plant and Actuator Transfer Function (G_combined):\n');
G_combined

% Plot the Root Locus of the combined system
figure;
rlocus(G_combined);
title('Root Locus of Combined Plant and Actuator (theta\_dot / command)');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;