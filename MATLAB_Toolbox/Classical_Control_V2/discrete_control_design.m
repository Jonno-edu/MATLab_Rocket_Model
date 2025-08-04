%% Chapter Almost: The Definitive Conservative Design
% This script uses the direct digital design method with a highly
% conservative bandwidth to ensure robustness against system phase lags.
% Inner loop: 200 Hz | Outer loop: 50 Hz

clc
clear
close all

%% --- 1. System Parameters & Constraints ---
% Vehicle physical properties
T = 27607;        % Thrust (N)
l_CG = 5.549;     % Nozzle to CG distance (m)
I_y = 21545.917;  % Pitch moment of inertia (kg*m^2)

% Actuator hardware model
omega_act = 62;     % Actuator natural frequency (rad/s)
zeta_act = 0.505;   % Actuator damping ratio

% Define the fast and slow sample times
Ts_inner = 1/200; % Inner loop runs at 200 Hz
Ts_outer = 1/50;  % Outer loop runs at 50 Hz

%% --- 2. Inner Loop (Fast Rate) Design ---
fprintf('--- Inner Loop Design (Ts = %.4f s, %d Hz) ---\n', Ts_inner, 1/Ts_inner);

% Create the full continuous-time plant model with aerodynamic instability
pitching_moment_derivative = 5500;
aero_gain = pitching_moment_derivative / I_y;
k_plant = T * l_CG / I_y;
plant_with_aero = tf([k_plant 0], [1 0 -aero_gain]);
actuator = tf([omega_act^2], [1 2*zeta_act*omega_act omega_act^2]);
plant_inner_open_loop_c = series(actuator, plant_with_aero);

% Discretize the plant at the FAST inner loop sample rate using Tustin
plant_inner_open_loop_d_fast = c2d(plant_inner_open_loop_c, Ts_inner, 'tustin');

% Use a HIGHLY conservative bandwidth for maximum robustness
bw_inner = omega_act / 5;
opts_inner = pidtuneOptions('PhaseMargin', 60);
[C_inner_d, ~] = pidtune(plant_inner_open_loop_d_fast, 'pidf', bw_inner, opts_inner);

% Create the fast closed-loop inner system
sys_inner_cl_d_fast = minreal(feedback(series(C_inner_d, plant_inner_open_loop_d_fast), 1));

disp('Tuned FAST Inner Loop PIDF Controller (200 Hz):');
disp(C_inner_d);


%% --- 3. Outer Loop (Slow Rate) Design ---
fprintf('\n--- Outer Loop Design (Ts = %.4f s, %d Hz) ---\n', Ts_outer, 1/Ts_outer);

% Resample the fast inner loop system to the slower outer loop sample rate
sys_inner_cl_resampled_slow = d2d(sys_inner_cl_d_fast, Ts_outer);

% The outer loop plant is the resampled inner loop plus an integrator at the SLOW rate
plant_outer_open_loop_d_slow = series(sys_inner_cl_resampled_slow, tf(1, [1 0], Ts_outer));

% Use a PI controller with a highly conservative phase margin
bw_outer = bw_inner / 3;
opts_outer = pidtuneOptions('PhaseMargin', 85);
[C_outer_d, ~] = pidtune(plant_outer_open_loop_d_slow, 'pi', bw_outer, opts_outer);

% Create the final, full autopilot system
sys_outer_cl_final = minreal(feedback(series(C_outer_d, plant_outer_open_loop_d_slow), 1));

disp('Tuned HIGHLY-ROBUST SLOW Outer Loop PI Controller (50 Hz):');
disp(C_outer_d);

%% --- 4. Final Performance and Stability Verification ---
fprintf('\n--- Final Autopilot Performance ---\n');
figure;
step(sys_outer_cl_final);
grid on;
title('Step Response of Final Multi-Rate Digital Autopilot');
stepinfo(sys_outer_cl_final)

% Definitive stability check for the final system
fprintf('\n--- Definitive Stability Check ---\n');
poles_final_system = pole(sys_outer_cl_final);
stability_margin = 1e-9; % A small tolerance for numerical precision

if all(abs(poles_final_system) < (1.0 - stability_margin))
    fprintf('SUCCESS: The final discrete-time system is STABLE.\n');
else
    fprintf('FAILURE: The final discrete-time system is UNSTABLE or MARGINALLY STABLE.\n');
end
disp('Poles of the final system (magnitudes should all be < 1):');
disp(abs(poles_final_system));
