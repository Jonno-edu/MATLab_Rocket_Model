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

%% Rocket Airframe State-Space Model (3 states)
% States: [alpha, theta, theta_dot]
A_plant = [-d1  0   1;
            0   0   1;
            d3  0   d4];
B_plant = [-d5;
            0;
            d2];
C_plant = eye(3); % Output all three states
D_plant = zeros(3,1);
sys_plant = ss(A_plant, B_plant, C_plant, D_plant);
fprintf('Rocket airframe model has %d states.\n', size(A_plant, 1));


%% Actuator State-Space Model (2 states)
% G_actuator(s) = (wn^2) / (s^2 + 2*zeta*wn*s + wn^2)
natural_frequency = 62; % rad/s
damping_ratio = 0.5858;
num_actuator = natural_frequency^2;
den_actuator = [1, 2*damping_ratio*natural_frequency, natural_frequency^2];
G_actuator = tf(num_actuator, den_actuator);
sys_actuator = ss(G_actuator);
fprintf('Actuator model has %d states.\n', size(sys_actuator.A, 1));


%% Create Augmented State-Space Model (Rocket + Actuator)
% This combines the actuator and rocket into one larger system.
% The controller's command goes to the actuator, and the actuator's
% output (nozzle angle) goes to the rocket airframe.
sys_aug = series(sys_actuator, sys_plant);

% Extract the augmented state-space matrices (A_aug, B_aug are what LQR needs)
[A_aug, B_aug, C_aug_full, D_aug_full] = ssdata(sys_aug);
fprintf('Augmented model has %d states (2 for actuator + 3 for rocket).\n\n', size(A_aug, 1));

% Note on state order: sys_aug states are ordered as
% [actuator_state_1; actuator_state_2; alpha; theta; theta_dot]
% so theta_dot is the 5th state.


%% Create SISO Transfer Function for Control System Designer
% We want the transfer function from the input (controller command)
% to the pitch rate (theta_dot).

% Define the output matrix for theta_dot.
% Assuming theta_dot is the 5th state in the augmented system.
C_siso_output = [0 0 0 0 1]; % Selects only the 5th state (theta_dot)

% D_siso_output will be zero as there's no direct feedthrough
D_siso_output = 0; 

% Create a new state-space system object that represents the
% augmented plant but only outputs theta_dot.
sys_aug_siso = ss(A_aug, B_aug, C_siso_output, D_siso_output);

% Convert this SISO state-space model to a transfer function
G_combined_siso = tf(sys_aug_siso);

fprintf('SISO Transfer Function (G_combined_siso = theta_dot / command):\n');
disp(G_combined_siso);


%% Launch Control System Designer
% You can now use this G_combined_siso (which is single-input, single-output)
% to design a controller using the graphical tools in Control System Designer.
fprintf('\nLaunching Control System Designer for G_combined_siso...\n');
controlSystemDesigner(G_combined_siso);

