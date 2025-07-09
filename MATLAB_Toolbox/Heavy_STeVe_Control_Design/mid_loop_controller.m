clear
clc
close all

% Plant Parameters (TVC Rocket @ t=66s)
% Physical parameters (SI units)
m = 1218.1;             % rocket mass (kg)
rho = 0.4;              % air density (kg/m³)
V = 397.9;              % velocity (m/s)
S = 0.200296;           % reference area (m²)
L = 9.542;              % reference length (m)
I = 2.29e+04;             % moment of inertia about pitch axis (kg⋅m²)
T = 2.67e+04;             % thrust (N)
L_arm = 3.9;            % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients
CNa = 3.5;              % Normal force coefficient derivative (1/rad), equivalent to CLa for small alpha
Cma = 29.661;            % Pitching moment coefficient derivative (1/rad) - positive = unstable
Cmq = -7.01;            % Pitch damping derivative (1/rad)

%% Step 1: Define Airframe-Only State-Space Model
% Here we define the A and B matrices for the 3-state rocket airframe.
fprintf('--- Defining 3-State Airframe Model ---\n');

% Calculate dimensional stability derivatives
q_bar = 0.5 * rho * V^2; % Dynamic pressure
Z_alpha = (q_bar * S / m) * CNa + (T / m);
Z_delta = T / m;
M_alpha = (q_bar * S * L / I) * Cma;
M_q = (q_bar * S * L^2) / (2 * V * I) * Cmq;
M_delta = (T * L_arm) / I;

% A and B matrices for the airframe
% States: [alpha; q; theta]
A_airframe = [-Z_alpha/V,  1,   0;
               M_alpha,    M_q, 0;
               0,          1,   0];

B_airframe = [-Z_delta/V;
               M_delta;
               0];

disp('A_airframe and B_airframe created.');
A_airframe
B_airframe

%% Step 2: Add Actuator Dynamics and Design LQR Controller
% The actuator model is added to the airframe model to design the
% full state feedback gain matrix K.
fprintf('\n--- Designing LQR Controller with Actuator Dynamics ---\n');

% Actuator State-Space Model (2 states)
natural_frequency = 62;
damping_ratio = 0.5858;
A_actuator = [0, 1; -natural_frequency^2, -2*damping_ratio*natural_frequency];
B_actuator = [0; natural_frequency^2];
C_actuator = [1, 0];
D_actuator = 0;

% Combine airframe and actuator models to form the 5-state system
A_combined = [A_airframe, B_airframe * C_actuator;
              zeros(2, 3), A_actuator];
B_combined = [B_airframe * D_actuator;
              B_actuator];

% LQR weight selection (Bryson's Rule)
max_alpha       = 0.5 * pi/180;
max_q           = 0.2 * pi/180;
max_theta       = 0.5 * pi/180;
max_act_pos     = 4 * pi/180;
max_act_rate    = 50 * pi/180;

max_TVC_command = 2 * pi/180;

Q_diag = [1/max_alpha^2, 1/max_q^2, 1/max_theta^2, 1/max_act_pos^2, 1/max_act_rate^2];
Q = diag(Q_diag);
R = 1/max_TVC_command^2;

% Calculate the LQR gain matrix K
K = lqr(A_combined, B_combined, Q, R);
K_lqr = K;


disp('LQR Gain Matrix K (for 5-state system):');
disp(K);

%% Step 3: Create Final Plant Model for Simulink
% This section creates the state-space matrices for the airframe-only model,
% but with horizontal acceleration added as a fourth output.
fprintf('\n--- Creating Final Plant Matrices for Simulink ---\n');

% The plant dynamics are the airframe-only dynamics
A_plant = A_airframe;
B_plant = B_airframe;

% Define the output equation for horizontal acceleration (a_h)
% a_h = (qS*CNa/m)*alpha - (T/m)*theta + (T/m)*delta_T
C_ah_row = [(q_bar * S * CNa) / m,  0,  -T / m];
D_ah_row = T / m;

% Construct the final C and D matrices
% Output vector will be [alpha; q; theta; a_h]
C_plant = [eye(3);      % Outputs the 3 states
           C_ah_row];   % Appends a_h as the 4th output

D_plant = [zeros(3,1);  % No direct feedthrough for the 3 states
           D_ah_row];   % Direct feedthrough for a_h

disp('The following matrices are ready for your Simulink state-space block:');
A_plant
B_plant
C_plant
D_plant
