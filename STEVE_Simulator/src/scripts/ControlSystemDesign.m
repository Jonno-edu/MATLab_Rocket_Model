% Control System Design for Rocket Pitch Control
% This script designs a cascaded control system for rocket pitch control
% with inner rate loop and outer angle loop, using the identified
% servo actuator model from system identification.

%clear; clc; close all;

% --- Configuration Switches ---
% Always using 2nd-order identified actuator model
actuator_model_type = 2;

% Set to true to generate plots, false to suppress plots
enable_plots = false; 

% --- Check for Control System Toolbox ---
if isempty(ver('control'))
    error('Control System Toolbox is required but not found.');
end
disp('Control System Toolbox found.');

% --- Parameters ---
F_thrust = 27.6e3 / 1;    % Thrust force per engine/actuator [N]
L = 4;                    % Moment arm [m]
I = 9470;                 % Moment of inertia [kg*m^2]

% --- Actuator Model Definitions ---
% ** First-Order Model Parameters (kept for reference) **
tau_act = 0.01; % Actuator time constant [s]
G_act_1st = tf(1, [tau_act 1]);

% ** Second-Order Model from System Identification **
% Replace the original model with our identified model
omega_n = 17.57;           % Natural frequency [rad/s] from identification
zeta = 0.77;               % Damping ratio from identification

% Convert degrees to radians for physical limits
max_deflection = 12.5 * pi/180;    % Maximum deflection [rad] (12.5 degrees)
min_deflection = -12.5 * pi/180;   % Minimum deflection [rad] (-12.5 degrees)
rate_limit = 75 * pi/180;          % Rate limit [rad/s] (75 deg/s)

% Our identified transfer function
G_act_2nd = tf([0.2133 6824], [1 272 3.091e04]);

% --- Select Actuator Model and Tuning Parameters Based on Switch ---
if actuator_model_type == 1
    G_act = G_act_1st;
    model_name = '1st Order';
    % Tuning parameters suitable for 1st order model
    target_bw_inner_hint = 30; % [rad/s]
    target_pm_inner = 45; % [degrees]
    target_bw_outer_hint = 3; % [rad/s]
    target_pm_outer = 60; % [degrees]
    disp('*** Tuning for 1st Order Actuator Model ***');
    fprintf(' - Time Constant (tau): %.3f s\n', tau_act);

elseif actuator_model_type == 2
    G_act = G_act_2nd;
    model_name = 'Identified 2nd Order';
    % Tuning parameters adjusted for identified model
    % Using ~1/5 of actuator natural frequency for inner loop bandwidth
    target_bw_inner_hint = 2; % [rad/s]
    target_pm_inner = 60; % [degrees]
    target_bw_outer_hint = 1; % [rad/s]
    target_pm_outer = 65; % [degrees]
    disp('*** Tuning for Identified 2nd Order Actuator Model ***');
    fprintf(' - Natural Frequency: %.2f rad/s (%.2f Hz)\n', omega_n, omega_n/(2*pi));
    fprintf(' - Damping Ratio: %.3f\n', zeta);
    % Display identified limits
    fprintf(' - Deflection Limits: ±%.2f rad (±%.2f deg)\n', max_deflection, max_deflection*180/pi);
    fprintf(' - Rate Limit: %.3f rad/s (%.2f deg/s)\n', rate_limit, rate_limit*180/pi);
else
    error('Invalid actuator_model_type selected. Choose 1 or 2.');
end
disp('Selected Actuator Model for Tuning:');
disp(G_act);

% --- Plant Modeling ---
s = tf('s');
G_pitchrate = (F_thrust * L / I) / s; % Pitch Dynamics (Torque -> Pitch Rate)
inner_plant_model = G_act * G_pitchrate; % Combined Inner Loop Plant
fprintf('\nInner Loop Plant Model (using %s actuator):\n', model_name);
disp(inner_plant_model);

% --- Inner Controller Design (Rate Loop) ---
fprintf('\nTuning Inner Loop PID Controller (Target PM: %.1f deg, BW Hint: %.1f rad/s)...\n', target_pm_inner, target_bw_inner_hint);
try
    options_inner = pidtuneOptions('PhaseMargin', target_pm_inner);
    inner_controller = pidtune(inner_plant_model, 'PID', target_bw_inner_hint, options_inner);
catch ME
    error('Failed to tune inner loop controller: %s', ME.message);
end
fprintf('Inner Loop PID Controller (tuned for %s actuator):\n', model_name);
disp(inner_controller); % Display controller object

% ** Extract Inner Controller Gains **
inner_Kp = inner_controller.Kp;
inner_Ki = inner_controller.Ki;
inner_Kd = inner_controller.Kd;
fprintf('   Extracted Inner Kp: %.6f\n', inner_Kp);
fprintf('   Extracted Inner Ki: %.6f\n', inner_Ki);
fprintf('   Extracted Inner Kd: %.6f\n', inner_Kd);

% --- Inner Loop Analysis ---
inner_open_loop = inner_controller * inner_plant_model; % Define open loop
inner_loop_closed = feedback(inner_open_loop, 1); % Define closed loop
fprintf('\nInner Closed-Loop TF (Rate Cmd -> Rate) (%s actuator):\n', model_name);
disp(inner_loop_closed);

if enable_plots
    figure_name_prefix = sprintf('%s Actuator (omega_n=%.2f)', model_name, omega_n); % More descriptive prefix

    figure('Name', sprintf('%s: Inner Loop Step Response', figure_name_prefix));
    step(inner_loop_closed);
    title(sprintf('Inner Loop Step Response (%s)', figure_name_prefix));
    xlabel('Time (s)'); ylabel('Pitch Rate (rad/s)'); grid on;

    figure('Name', sprintf('%s: Inner Loop Margins', figure_name_prefix));
    margin(inner_open_loop);
    title(sprintf('Inner Loop Open-Loop Margins (%s)', figure_name_prefix));
    grid on;
end % end enable_plots for basic inner analysis

% Always calculate margins even if not plotting
[Gm_inner, Pm_inner, Wcg_inner, Wcp_inner] = margin(inner_open_loop);
fprintf('\n--- Inner Loop Stability Margins (%s) ---\n', model_name);
fprintf('Achieved Gain Margin (Gm): %.2f dB (at %.2f rad/s)\n', 20*log10(Gm_inner), Wcg_inner);
fprintf('Achieved Phase Margin (Pm): %.2f deg (at %.2f rad/s)\n', Pm_inner, Wcp_inner);

if enable_plots
    % --- Inner Loop Root Locus and Pole-Zero Map ---
    figure('Name', sprintf('%s: Inner Loop Locus & Poles', figure_name_prefix));
    subplot(1, 2, 1);
    rlocus(inner_open_loop);
    title('Inner Loop: Open Loop Root Locus');
    grid on;
    hold on; % Add current poles
    current_poles_inner_ol = pole(inner_loop_closed);
    plot(real(current_poles_inner_ol), imag(current_poles_inner_ol), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;
    try % Add legend, but catch potential errors
        legend('Locus Paths', 'Closed-Loop Poles', 'Location', 'best');
    catch E
        warning('Could not add legend to Inner Loop Root Locus plot: %s', E.message);
    end

    subplot(1, 2, 2);
    pzmap(inner_loop_closed);
    title('Inner Loop: Closed Loop Pole-Zero Map');
    grid on;
    sgtitle(sprintf('Inner (Rate) Loop Analysis (%s)', figure_name_prefix)); % Overall title
end % end enable_plots for inner locus

% --- Outer Plant Definition (Angle Loop) ---
outer_plant = inner_loop_closed * (1 / s);
fprintf('\nOuter Loop Plant TF (Angle Cmd -> Angle) (%s actuator):\n', model_name);
disp(outer_plant);

% --- Outer Controller Design (Angle Loop) ---
fprintf('\nTuning Outer Loop PI Controller (Target PM: %.1f deg, BW Hint: %.1f rad/s)...\n', target_pm_outer, target_bw_outer_hint);
try
    options_outer = pidtuneOptions('PhaseMargin', target_pm_outer);
    angle_controller = pidtune(outer_plant, 'PI', target_bw_outer_hint, options_outer);
catch ME
    error('Failed to tune outer loop controller: %s', ME.message);
end
fprintf('Outer Loop PI Controller (tuned for %s actuator plant):\n', model_name);
disp(angle_controller); % Display controller object

% ** Extract Outer Controller Gains **
outer_Kp = angle_controller.Kp;
outer_Ki = angle_controller.Ki;
outer_Kd = 0; % Explicitly set to 0 for PI controller
fprintf('   Extracted Outer Kp: %.6f\n', outer_Kp);
fprintf('   Extracted Outer Ki: %.6f\n', outer_Ki);
fprintf('   Extracted Outer Kd: %.6f (Set to 0 as it''s a PI controller)\n', outer_Kd);

% --- Outer Loop Analysis ---
outer_open_loop = angle_controller * outer_plant; % Define open loop
outer_cl = feedback(outer_open_loop, 1); % Define closed loop
fprintf('\nOuter Closed-Loop TF (Angle Cmd -> Angle) (%s actuator):\n', model_name);
disp(outer_cl);

if enable_plots
    figure('Name', sprintf('%s: Outer Loop Step Response', figure_name_prefix));
    step(outer_cl);
    title(sprintf('Outer Loop Step Response (%s)', figure_name_prefix));
    xlabel('Time (s)'); ylabel('Pitch Angle (rad)'); grid on;

    figure('Name', sprintf('%s: Outer Loop Bode Plot', figure_name_prefix));
    bode(outer_cl);
    title(sprintf('Outer Loop Closed-Loop Bode Plot (%s)', figure_name_prefix));
    grid on;

    figure('Name', sprintf('%s: Outer Loop Margins', figure_name_prefix));
    margin(outer_open_loop);
    title(sprintf('Outer Loop Open-Loop Gain and Phase Margins (%s)', figure_name_prefix));
    grid on;
end % end enable_plots for basic outer analysis

% Always calculate margins even if not plotting
[Gm, Pm, Wcg, Wcp] = margin(outer_open_loop);
fprintf('\n--- Outer Loop Stability Margins (%s) ---\n', model_name);
fprintf('Achieved Gain Margin (Gm): %.2f dB (at %.2f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Achieved Phase Margin (Pm): %.2f deg (at %.2f rad/s)\n', Pm, Wcp);

if enable_plots
    % --- Outer Loop Root Locus and Pole-Zero Map ---
    figure('Name', sprintf('%s: Outer Loop Locus & Poles', figure_name_prefix));
    subplot(1, 2, 1);
    rlocus(outer_open_loop);
    title('Outer Loop: Open Loop Root Locus');
    grid on;
    hold on; % Overlay current poles
    current_poles_outer = pole(outer_cl);
    plot(real(current_poles_outer), imag(current_poles_outer), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;
     try % Add legend, but catch potential errors
        legend('Locus Paths', 'Closed-Loop Poles', 'Location', 'best');
    catch E
        warning('Could not add legend to Outer Loop Root Locus plot: %s', E.message);
    end

    subplot(1, 2, 2);
    pzmap(outer_cl);
    title('Outer Loop: Closed Loop Pole-Zero Map');
    grid on;
    sgtitle(sprintf('Outer (Angle) Loop Analysis (%s)', figure_name_prefix)); % Overall title
end % end enable_plots for outer locus

% --- Note on Rate and Deflection Limits ---
fprintf('\n--- Actuator Limits Note ---\n');
fprintf('The identified servo actuator has the following constraints:\n');
fprintf(' - Deflection limits: ±%.2f rad (±%.2f deg)\n', max_deflection, max_deflection*180/pi);
fprintf(' - Rate limit: %.3f rad/s (%.2f deg/s)\n', rate_limit, rate_limit*180/pi);
fprintf('For simulation testing with this actuator:\n');
fprintf(' 1. Ensure anti-windup is enabled in the PID implementation to prevent integrator windup.\n');
fprintf(' 2. Monitor actuator commands to check for saturation (especially rate limits).\n');
fprintf(' 3. Consider that the actuator may be the performance bottleneck (natural frequency %.2f rad/s).\n', omega_n);

% --- Create Full System Model with Nonlinear Actuator ---
fprintf('\n--- Full System Model Notes ---\n');
fprintf('For Simulink implementation, model the servo as:\n');
fprintf(' 1. Linear part: Transfer function [0.02296 s + 68.16]/[s^2 + 27.16 s + 308.7]\n');
fprintf(' 2. Nonlinear constraints: Saturation block (±%.2f rad) and Rate Limiter (±%.2f rad/s)\n', max_deflection, rate_limit);
fprintf(' 3. Pushrod linkage: Convert servo angle to nozzle angle using leverage ratio\n');

% --- GAINS FOR SIMULATION (Always Displayed) ---
fprintf('\n--- GAINS FOR SIMULATION ---\n');
fprintf('Inner Kp: %.6f\n', inner_Kp);
fprintf('Inner Ki: %.6f\n', inner_Ki);
fprintf('Inner Kd: %.6f\n', inner_Kd);
fprintf('Outer Kp: %.6f\n', outer_Kp);
fprintf('Outer Ki: %.6f\n', outer_Ki);
fprintf('Outer Kd: %.6f\n', outer_Kd); % This will be 0 for PI controller
fprintf('---------------------------\n');

% --- Additional Notes on Implementation ---
fprintf('\n--- Implementation Recommendations ---\n');
fprintf('1. The identified actuator bandwidth (%.2f rad/s) is much lower than the\n', omega_n);
fprintf('   original actuator in the script (500 rad/s), which may result in slower\n');
fprintf('   overall control performance.\n');
fprintf('2. Consider using the nozzle angle kinematic conversion function in Simulink:\n');
fprintf('   function theta_n = servo2nozzle(theta_s)\n');
fprintf('       L_servo = 0.02; %% 20mm servo arm\n');
fprintf('       L_nozzle = 0.08; %% 80mm bell crank\n');
fprintf('       theta_n = asin((L_servo/L_nozzle) * sin(theta_s));\n');
fprintf('   end\n');
fprintf('3. The tuned PID values above account for the servo dynamics but not the\n');
fprintf('   pushrod kinematics. Use these gains with the full nonlinear model.\n');
