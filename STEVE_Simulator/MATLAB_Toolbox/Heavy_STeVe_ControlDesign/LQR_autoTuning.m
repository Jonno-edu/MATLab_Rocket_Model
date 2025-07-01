%% Autopilot Design via Direct Pole Placement (Feedback Only)

clear
clc
close all

%% 1. System Model Definition
% Physical parameters
m = 1277.9; rho = 0.5; V = 340.8; S = 0.200296; L = 9.542;
I = 2.38e4; T = 2.64e4; L_arm = 3.9;

% Aerodynamic coefficients
CLa = 2.0; Cma = 0.885; Cmq = -1.05;

% Derived constants
q = 0.5*rho*V^2; d1 = (rho*V*S)/(2*m)*CLa+T/(m*V); d2 = T*L_arm/I;
d3 = (rho*V^2*S*L)/(2*I)*Cma; d4 = (rho*V*S*L^2/(2*I))*Cmq; d5 = T/(m*V);

% State-Space Models
A_plant = [-d1 0 1; 0 0 1; d3 0 d4];
B_plant = [-d5; 0; d2];
sys_plant = ss(A_plant, B_plant, eye(3), 0);
wn = 62; zeta = 0.5858;
A_act = [0 1; -wn^2 -2*zeta*wn];
B_act = [0; wn^2];
sys_actuator = ss(A_act, B_act, [1 0], 0);
sys_open_loop = series(sys_plant, sys_actuator);
A = sys_open_loop.A;
B = sys_open_loop.B;

%% 2. Controller Design via Pole Placement
fprintf('Designing controller using direct pole placement...\n');

% Define the desired closed-loop pole locations.
% We need 5 poles for our 5-state system. All must be faster than -0.5.
p1 = -1.0;
p2 = -1.5;
p3 = -2.0;

% Define a complex conjugate pair for good damping
zeta_d = 0.8; % Desired damping ratio
wn_d = 4;     % Desired natural frequency
p4 = -zeta_d*wn_d + 1i*wn_d*sqrt(1-zeta_d^2);
p5 = -zeta_d*wn_d - 1i*wn_d*sqrt(1-zeta_d^2);

desired_poles = [p1; p2; p3; p4; p5];

fprintf('Desired Pole Locations:\n');
disp(desired_poles);

% Use the `place` command to calculate the gain matrix K
% This command can fail if the system is not controllable.
try
    K = place(A, B, desired_poles);
catch ME
    if (strcmp(ME.identifier,'Control:design:Uncontrollable'))
        error('FATAL: The system is not controllable. Pole placement is not possible.');
    else
        rethrow(ME);
    end
end

fprintf('Success! Pole placement gain matrix K calculated.\n\n');
disp('Gain Matrix K:');
disp(K);

%% 3. Verify Controller Performance
% Verify that the closed-loop poles are where we placed them.
A_closed_loop = A - B*K;
actual_poles = eig(A_closed_loop);
fprintf('Actual Closed-Loop Pole Locations:\n');
disp(actual_poles);

% Create the closed-loop system (with no external input for now)
sys_closed_loop = ss(A_closed_loop, zeros(5,1), eye(5), 0);

% Analyze how the controller handles an initial disturbance.
% Let's say the rocket is perturbed by a small angle of attack.
x0 = [0.1; 0; 0; 0; 0]; % Initial condition: alpha = 0.1 rad

figure('Name', 'Pole Placement Disturbance Rejection');
initial(sys_closed_loop, x0);
grid on;
title('System Response to an Initial Disturbance');
xlabel('Time (s)');
ylabel('State Values');


K_lqr = K;

