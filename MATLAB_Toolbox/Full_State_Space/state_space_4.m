% Rocket Longitudinal 4-State LQR Example

% Parameters
m = 82.9;           % Rocket mass (kg)
Jt = 30.0;          % Transverse inertia (kg*m^2)
T = 12000;          % Thrust (N)
l = 1.5;            % TVC moment arm (m)
q_dyn = 5000;       % Dynamic pressure (N/m^2)
S = 0.045;          % Ref. area (m^2)
d = 0.24;           % Ref. diameter (m)
g = 9.81;           % Gravity (m/s^2)
CA = 0.5;
CM = 0.2;

% Trim
u0 = 500;
w0 = 0;
q0 = 0;
theta0 = deg2rad(5);

% State-space matrices
A = zeros(4,4);
A(1,1) = -q_dyn*CA*S/m;
A(1,2) = q0;
A(1,3) = w0;
A(1,4) = -g * sin(theta0);

A(2,1) = -q0;
A(2,2) = -q_dyn*CA*S/m;
A(2,3) = -u0;
A(2,4) = g * cos(theta0);

A(3,2) = 1.0;
A(3,3) = -q_dyn*CM*S*d/Jt;

A(4,3) = 1.0;

B = zeros(4,1);
B(3) = T*l/Jt;    % TVC pitch input

C = eye(4);
D = zeros(4,1);

% Create state-space system
sys = ss(A, B, C, D);

%% Plot open-loop step response for pitch angle (unstable)
t = 0:0.01:5;
u_input = deg2rad(2) * ones(length(t),1); % 2 deg TVC step
[y_ol, t_ol, x_ol] = lsim(sys, u_input, t);

figure;
plot(t_ol, rad2deg(y_ol(:,4)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Open-Loop Pitch Angle Response (Unstable)');
grid on;

%% LQR Controller Design

Q = diag([1, 1, 10, 100]); % Penalize pitch rate and especially pitch angle
R = 1;                     % Penalize control input magnitude

K = lqr(A, B, Q, R);

% Closed-loop A matrix
Acl = A - B * K;
sys_cl = ss(Acl, B, C, D);

% Compute open-loop poles
p_ol = eig(A);

% Closed-loop A matrix
Acl = A - B * K;

% Compute closed-loop poles
p_cl = eig(Acl);

% Plot open-loop and closed-loop pole maps
figure;
plot(real(p_ol), imag(p_ol), 'rx', 'MarkerSize', 12, 'LineWidth', 2); hold on;
plot(real(p_cl), imag(p_cl), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Real Part');
ylabel('Imaginary Part');
title('Pole Map: Open-loop (red X) vs Closed-loop (blue O)');
legend('Open-loop Poles','Closed-loop Poles');
grid on;

