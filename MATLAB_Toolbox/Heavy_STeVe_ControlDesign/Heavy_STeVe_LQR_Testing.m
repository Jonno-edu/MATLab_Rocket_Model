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

%% Combine Models
% Connect actuator and plant in series to form the open-loop system.
% The signal flow is: command -> actuator -> plant.
% The order of multiplication is sys_plant * sys_actuator.
sys_open_loop = sys_plant * sys_actuator;

fprintf('Combined open-loop model has %d states.\n', size(sys_open_loop.A, 1));

% Extract A and B matrices for controller design
A = sys_open_loop.A;
B = sys_open_loop.B;

%% LQR Controller Design (Bryson's Rule)
% Define maximum acceptable deviations for each state and input.
% State order is: [alpha, theta, theta_dot, act_rate, act_pos]
max_alpha     = 3*pi/180; 
max_theta     = 3*pi/180;  
max_theta_dot = 50*pi/180;  
max_act_rate  = 50*pi/180;    
max_act_pos   = 4*pi/180;   

max_TVC_command = 4*pi/180;

% Construct Q matrix using Bryson's Rule
Q_diag = [1/max_alpha^2, 1/max_theta^2, 1/max_theta_dot^2, 1/max_act_rate^2, 1/max_act_pos^2];
Q = diag(Q_diag);

% Construct R matrix using Bryson's Rule
R = 1/max_TVC_command^2;

% Calculate the LQR gain matrix K
K_lqr = lqr(A, B, Q, R);

disp('LQR Gain Matrix K:');
disp(K_lqr);
