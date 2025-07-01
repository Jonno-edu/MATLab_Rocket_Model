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
CLa = 2.0; Cma = 0.885; Cmq = -1.05;

% Calculate derived constants
q = 0.5 * rho * V^2;
d1 = (rho*V*S)/(2*m)*CLa + T/(m*V); d2 = T*L_arm/I;
d3 = (rho*V^2*S*L)/(2*I)*Cma; d4 = (rho*V*S*L^2)/(2*I)*Cmq;
d5 = T/(m*V);

%% Rocket Airframe and Actuator Models
A_plant = [-d1 0 1; 0 0 1; d3 0 d4];
B_plant = [-d5; 0; d2];
% We need to define C to select the output we want to track (alpha)
C_plant = [1 0 0]; % Output is alpha
D_plant = 0;
sys_plant = ss(A_plant, B_plant, C_plant, D_plant);

natural_frequency = 62; damping_ratio = 0.5858;
G_actuator = tf(natural_frequency^2, [1, 2*damping_ratio*natural_frequency, natural_frequency^2]);
sys_actuator = ss(G_actuator);

%% Combine Models
sys_open_loop = sys_plant * sys_actuator;
A = sys_open_loop.A;
B = sys_open_loop.B;
C = sys_open_loop.C; % We only have one output now: alpha
D = sys_open_loop.D;

%% LQR Controller Design (for the original 5-state system)
% This is the feedback gain K for stabilization.
% State order: [alpha, theta, theta_dot, act_rate, act_pos]
max_alpha     = 2*pi/180; 
max_theta     = 15*pi/180;  
max_theta_dot = 2*pi/180;  
max_act_rate  = 50*pi/180;    
max_act_pos   = 4*pi/180;   

max_TVC_command = 0.5*pi/180;

% Construct Q and R matrices
Q = diag([1/max_alpha^2, 1/max_theta^2, 1/max_theta_dot^2, 1/max_act_rate^2, 1/max_act_pos^2]);
R = 1/max_TVC_command^2;

% Calculate the LQR feedback gain matrix K
K_lqr = lqr(A, B, Q, R);
disp('LQR Feedback Gain Matrix K (5 states):');
disp(K_lqr);

eig(A - B*K_lqr)


%% Feedforward Gain Calculation (for tracking)
% This method calculates the DC gain of the STABILIZED closed-loop system.
% This is the robust way to handle plants with integrators.
% The closed-loop system is y = C*inv(sI - (A-B*K))*B*u_scaled
dc_gain = C * inv(-(A - B*K_lqr)) * B;

% Nbar is the inverse of the DC gain.
Nbar = 1/dc_gain;

disp('Feedforward Gain Nbar:');
disp(Nbar);
