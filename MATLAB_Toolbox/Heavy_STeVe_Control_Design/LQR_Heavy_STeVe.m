clear
clc
close all

%% Plant Parameters (TVC Rocket)
% This script models the rocket at a single point in time to design and
% test a single-point LQR controller.

% Physical parameters (SI units)
m = 1873.04;             % rocket mass (kg)
rho = 1.2250;              % air density (kg/m³)
V = 1.55;              % velocity (m/s)
S = 0.200296;           % reference area (m²)
L = 9.542;              % reference length (m)
I = 33017;             % moment of inertia about pitch axis (kg⋅m²)
T = 33017;             % thrust (N)
L_arm = 4.016;            % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients
CNa = -2;              % Normal force coefficient derivative (1/rad)
Cma = 16.714;            % Pitching moment coefficient derivative (1/rad) - positive = unstable
Cmq = -4;               % Pitch damping derivative (1/rad) - Should be negative for damping

%% Dimensional Stability Derivatives
% Calculate dimensional derivatives based on the corrected state-space derivation.
q_bar = 0.5 * rho * V^2; % Dynamic pressure

Z_alpha = (q_bar * S / m) * CNa;
Z_delta = T / m;
M_alpha = (q_bar * S * L / I) * Cma;
M_q = (q_bar * S * L^2) / (2 * V * I) * Cmq;
M_delta = (T * L_arm) / I;

%% Rocket Airframe State-Space Model (3 states)
% States are now correctly defined as: [alpha; q; theta]
A_plant = [-Z_alpha/V,  1,   0;
            M_alpha,    M_q, 0;
            0,          1,   0];

B_plant = [-Z_delta/V;
            M_delta;
            0];

C_plant = eye(3);
D_plant = zeros(3,1);

sys_plant = ss(A_plant, B_plant, C_plant, D_plant);
sys_plant.StateName = {'alpha', 'q', 'theta'};
sys_plant.InputName = {'delta_T'};
sys_plant.OutputName = {'alpha_out', 'q_out', 'theta_out'};

fprintf('Rocket airframe model has %d states.\n', size(A_plant, 1));
fprintf('Open-loop plant poles (eigenvalues of A_plant):\n');
disp(eig(A_plant));

%% Actuator State-Space Model (2 states)
natural_frequency = 62;       % rad/s
damping_ratio = 0.5858;
num_actuator = natural_frequency^2;
den_actuator = [1, 2*damping_ratio*natural_frequency, natural_frequency^2];
G_actuator = tf(num_actuator, den_actuator);

sys_actuator = ss(G_actuator);
sys_actuator.StateName = {'act_pos', 'act_rate'};
sys_actuator.InputName = {'delta_cmd'};
sys_actuator.OutputName = {'delta_actual'};

fprintf('Actuator model has %d states.\n', size(sys_actuator.A, 1));

%% Combine Models
sys_open_loop = series(sys_actuator, sys_plant);

fprintf('Combined open-loop model has %d states.\n', size(sys_open_loop.A, 1));

% Extract A and B matrices for controller design
A = sys_open_loop.A;
B = sys_open_loop.B;

%% LQR Controller Design (Bryson's Rule) - ROBUST METHOD

% --- THIS IS THE CORRECTED SECTION ---
% First, define maximum acceptable deviations for each state.
max_devs = struct();
max_devs.act_pos     = 4 * pi/180;     % rad
max_devs.act_rate    = 50 * pi/180;    % rad/s
max_devs.alpha       = 1 * pi/180;     % rad
max_devs.q           = 20 * pi/180;    % rad/s (pitch rate)
max_devs.theta       = 1 * pi/180;   % rad

max_TVC_command = 0.1 * pi/180;     % rad

% Second, programmatically get the ACTUAL state order from the combined system.
actual_state_order = sys_open_loop.StateName;
fprintf('Actual combined system state order is:\n');
disp(actual_state_order);

% Third, build the Q_diag vector in the CORRECT order by looping through
% the actual state names and assigning the corresponding penalty.
num_states = length(actual_state_order);
Q_diag = zeros(1, num_states);
for i = 1:num_states
    state_name = actual_state_order{i};
    if isfield(max_devs, state_name)
        Q_diag(i) = 1 / max_devs.(state_name)^2;
    else
        error('State name "%s" not found in max_devs structure.', state_name);
    end
end

fprintf('\nQ diagonal constructed in the correct order to match the system states.\n');
Q = diag(Q_diag);

% Construct R matrix using Bryson's Rule
R = 1/max_TVC_command^2;

% Calculate the LQR gain matrix K
K_lqr = lqr(A, B, Q, R);

disp('LQR Gain Matrix K:');
disp(K_lqr);

disp('Eigenvalues of the closed-loop system (A - B*K):');
disp(eig(A - B*K_lqr));
