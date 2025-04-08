% Control System Design for Rocket Pitch Control
% This script designs a cascaded control system for rocket pitch control
% with inner rate loop and outer angle loop using a 1st-order actuator model.

clear; clc; close all;

% --- Check for Control System Toolbox ---
if isempty(ver('control'))
    error('Control System Toolbox is required but not found.');
end
disp('Control System Toolbox found.');

% --- Parameters ---
F_thrust = 27.6e3 / 2;    % Thrust force per engine/actuator [N] (Adjust if total)
L = 4;                  % Moment arm [m]
I = 9470;               % Moment of inertia [kg·m²]

% --- Actuator Model Selection ---
% Using a 1st Order Linear Actuator Model
tau_act = 0.01; % Actuator time constant [s] - ADJUST AS NEEDED
G_act = tf(1, [tau_act 1]); % First-order actuator model (Gain=1, Time Constant=tau_act)
disp('Selected Actuator Model (1st Order):');
fprintf(' - Time Constant (tau): %.3f s\n', tau_act);
G_act

% --- Plant Modeling ---
s = tf('s');

% Pitch Dynamics (Torque -> Pitch Rate)
% Torque = F_thrust * L * (actuator_output_signal) assuming actuator output is normalized deflection or similar
% Angular Acceleration = Torque / I
% Angular Rate = Integral(Angular Acceleration) = Torque / (I * s)
G_pitchrate = (F_thrust * L / I) / s; % Maps effective torque signal to pitch rate

% Combined Plant Model for Inner Loop (Actuator Cmd -> Pitch Rate)
inner_plant_model = G_act * G_pitchrate;
disp('Inner Loop Plant Model (using 1st order actuator):');
inner_plant_model

% --- Inner Controller Design (Rate Loop) ---
target_bw_inner = 7.5; % Target bandwidth [rad/s] (Adjust based on performance needs)
fprintf('Tuning Inner Loop PID Controller (Target Bandwidth: %.1f rad/s)...\n', target_bw_inner);
try
    % Tune PID for the inner loop (rate control)
    inner_controller = pidtune(inner_plant_model, 'PID', target_bw_inner);
catch ME
    error('Failed to tune inner loop controller: %s', ME.message);
end
disp('Inner Loop PID Controller (tuned for 1st order actuator):');
inner_controller

% --- Inner Loop Analysis ---
% Calculate Inner Closed-Loop TF (Rate Command -> Actual Rate)
inner_loop_closed = feedback(inner_controller * inner_plant_model, 1);
disp('Inner Closed-Loop TF (Rate Cmd -> Rate):');
inner_loop_closed

% Plot inner loop step response
figure('Name', 'Inner Loop Step Response (1st Order Actuator)');
step(inner_loop_closed);
title('Inner Loop Step Response (Rate Cmd -> Actual Rate)');
xlabel('Time (s)');
ylabel('Pitch Rate (rad/s)');
grid on;

% --- Outer Plant Definition (Angle Loop) ---
% Outer plant is the inner closed-loop rate response integrated
outer_plant = inner_loop_closed * (1 / s);
disp('Outer Loop Plant TF (Angle Cmd -> Angle):');
outer_plant

% --- Outer Controller Design (Angle Loop) ---
target_bw_outer = 0.5; % Target bandwidth [rad/s] (Must be slower than inner loop, e.g., factor of 5-10)
fprintf('Tuning Outer Loop PI Controller (Target Bandwidth: %.1f rad/s)...\n', target_bw_outer);
try
    % Tune PI for the outer loop (angle control) - PI is common here
    angle_controller = pidtune(outer_plant, 'PI', target_bw_outer);
catch ME
    error('Failed to tune outer loop controller: %s', ME.message);
end
disp('Outer Loop PI Controller (tuned for 1st order actuator plant):');
angle_controller

% --- Outer Loop Analysis ---
% Calculate Final Outer Closed-Loop TF (Angle Command -> Actual Angle)
outer_cl = feedback(angle_controller * outer_plant, 1);
disp('Outer Closed-Loop TF (Angle Cmd -> Angle):');
outer_cl

% Plot outer loop step response
figure('Name', 'Outer Loop Step Response (1st Order Actuator)');
step(outer_cl);
title('Outer Loop Step Response (Angle Cmd -> Actual Angle)');
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
grid on;

% Plot Bode diagram of the closed loop
figure('Name', 'Outer Loop Bode Plot (1st Order Actuator)');
bode(outer_cl);
title('Outer Loop Closed-Loop Bode Plot');
grid on;

% Calculate and Plot Open-Loop Margins for the Outer Loop
outer_open_loop = angle_controller * outer_plant;
figure('Name', 'Outer Loop Margins (1st Order Actuator)');
margin(outer_open_loop);
title('Outer Loop Open-Loop Gain and Phase Margins');
grid on;

% Display margins
[Gm, Pm, Wcg, Wcp] = margin(outer_open_loop);
fprintf('\n--- Outer Loop Stability Margins ---\n');
fprintf('Gain Margin (Gm): %.2f dB (at %.2f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Phase Margin (Pm): %.2f deg (at %.2f rad/s)\n', Pm, Wcp);

% Plot root locus of the compensated outer loop
figure('Name', 'Outer Loop Root Locus (1st Order Actuator)');
rlocus(outer_open_loop);
title('Root Locus of Compensated Outer Loop');
grid on;

% --- Note on Rate Limits ---
max_rate_deg_s = 150;
max_rate_rad_s = max_rate_deg_s * pi / 180;
fprintf('\n--- Actuator Rate Limit Note ---\n');
fprintf('The actuator physical rate limit of %.1f deg/s (%.3f rad/s) is a non-linearity.\n', max_rate_deg_s, max_rate_rad_s);
fprintf('This linear design does not include saturation effects.\n');
fprintf('The designed controller gains (Kp, Ki, Kd) should be used in a simulation environment (like Simulink)\n');
fprintf('that includes a rate limiter or saturation block on the actuator output to evaluate performance under saturation.\n');
fprintf('The main effect of the rate limit will be on the maximum slew rate for large commands.\n');