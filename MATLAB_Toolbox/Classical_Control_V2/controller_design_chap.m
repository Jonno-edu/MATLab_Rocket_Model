%% Chapter Almost: Rocket Pitch Control Design & Analysis
% This script walks through the design of a cascaded pitch control system
% for a finless TVC rocket, from defining the plant to tuning and
% analyzing the inner (pitch rate) and outer (pitch angle) control loops.
% This version includes extraction of the derivative filter coefficient (N).

% Clear workspace and close all figures
clc
clear
close all

%% --- 1. System Parameters & Constraints ---
% Vehicle physical properties
T = 27607;        % Thrust (N)
l_CG = 5.55;     % Nozzle to CG distance (m)
I_y = 21546;      % Pitch moment of inertia (kg*m^2)

% Actuator hardware model
omega_act = 62;     % Actuator natural frequency (rad/s)
zeta_act = 0.505;   % Actuator damping ratio

%% --- 2. Plant Definition ---
% Define the open-loop dynamics of the rocket pitch rate.

% Aerodynamic model
pitching_moment_derivative = 5500; % Aerodynamic instability (Nm/rad)
aero_gain = pitching_moment_derivative / I_y;

% Control authority
k_plant = T * l_CG / I_y;

% Create transfer function models
plant_with_aero = tf([k_plant 0], [1 0 -aero_gain]); % Rocket pitch rate dynamics
actuator = tf([omega_act^2], [1 2*zeta_act*omega_act omega_act^2]);

% Combine models to create the full open-loop plant for the inner loop
plant_inner_open_loop = series(actuator, plant_with_aero);

%% --- 3. Inner Loop (Pitch Rate) Controller Design ---
% Design a fast PID controller to stabilize the pitch rate.
fprintf('--- Inner Loop (Pitch Rate) Design ---\n'); %[output:3cc24c63]

% Design Target: 1/3 of actuator bandwidth
bw_inner = omega_act / 3;
fprintf('Target Inner Loop Bandwidth: %.2f rad/s\n', bw_inner); %[output:3fe6878a]

% Tune the PID controller
opts_inner = pidtuneOptions('PhaseMargin', 60);
[C_inner, info_inner] = pidtune(plant_inner_open_loop, 'pid', bw_inner, opts_inner); % Capture the info struct

% Create the closed-loop inner system
sys_inner_cl = feedback(series(C_inner, plant_inner_open_loop), 1);

% Display the controller gains and the CRITICAL filter coefficient
disp('Tuned Inner Loop PID Controller (C_inner):'); %[output:331fcf1d]
disp(C_inner); %[output:153a3311]
fprintf('CRITICAL ---> Derivative Filter Coefficient (N): %.2f\n', C_inner.N); %[output:51be2b41]

%% --- 4. Outer Loop (Pitch Angle) Controller Design ---
% (The rest of the script remains the same)
fprintf('\n--- Outer Loop (Pitch Angle) Design ---\n');
bw_outer = bw_inner / 3;
fprintf('Target Outer Loop Bandwidth: %.2f rad/s\n', bw_outer);
plant_outer_open_loop = series(sys_inner_cl, tf(1,[1 0]));
opts_outer = pidtuneOptions('PhaseMargin', 60);
[C_outer, ~] = pidtune(plant_outer_open_loop, 'p', bw_outer, opts_outer);
sys_outer_cl = minreal(feedback(series(C_outer, plant_outer_open_loop), 1));
disp('Tuned Outer Loop P Controller (C_outer):');
disp(C_outer);

%% --- 5. Final Performance and Stability Verification ---
fprintf('\n--- Final Autopilot Performance ---\n');
figure;
step(sys_outer_cl);
grid on;
title('Step Response of Full Autopilot (Pitch Angle Control)');
stepinfo(sys_outer_cl)

fprintf('\n--- Definitive Stability Check ---\n');
poles_final_system = pole(sys_outer_cl);
if all(real(poles_final_system) < 0)
    fprintf('SUCCESS: The final closed-loop system is STABLE.\n');
else
    fprintf('FAILURE: The final closed-loop system is UNSTABLE.\n');
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":45.8}
%---
%[output:3cc24c63]
%   data: {"dataType":"text","outputData":{"text":"--- Inner Loop (Pitch Rate) Design ---\n","truncated":false}}
%---
%[output:3fe6878a]
%   data: {"dataType":"text","outputData":{"text":"Target Inner Loop Bandwidth: 20.67 rad\/s\n","truncated":false}}
%---
%[output:331fcf1d]
%   data: {"dataType":"text","outputData":{"text":"Tuned Inner Loop PID Controller (C_inner):\n","truncated":false}}
%---
%[output:153a3311]
%   data: {"dataType":"text","outputData":{"text":"  <a href=\"matlab:helpPopup('pid')\" style=\"font-weight:bold\">pid<\/a> with properties:\n\n              Kp: 2.6207\n              Ki: 9.4277\n              Kd: 0.0646\n              Tf: 0\n        IFormula: ''\n        DFormula: ''\n      InputDelay: 0\n     OutputDelay: 0\n       InputName: {''}\n       InputUnit: {''}\n      InputGroup: [1×1 struct]\n      OutputName: {''}\n      OutputUnit: {''}\n     OutputGroup: [1×1 struct]\n           Notes: [0×1 string]\n        UserData: []\n            Name: ''\n              Ts: 0\n        TimeUnit: 'seconds'\n    SamplingGrid: [1×1 struct]\n\n","truncated":false}}
%---
%[output:51be2b41]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Ambiguous pid property: 'N'."}}
%---
