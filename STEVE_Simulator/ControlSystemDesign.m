% Control System Design for Rocket Pitch Control
% This script designs a cascaded control system for rocket pitch control
% with inner rate loop and outer angle loop, allowing selection between
% 1st-order and 2nd-order actuator models for tuning.
% It extracts controller gains into variables for simulation use and
% allows disabling plots for batch runs.

%clear; clc; close all; % Keep this clear for the tuning script itself

% --- Configuration Switches ---
% Set to 1 for 1st-order actuator model tuning
% Set to 2 for 2nd-order actuator model tuning
actuator_model_type = 2; % <<< CHANGE THIS VALUE (1 or 2)

% Set to true to generate plots, false to suppress plots
enable_plots = false; % <<< CHANGE THIS VALUE (true or false)

% --- Check for Control System Toolbox ---
if isempty(ver('control'))
    error('Control System Toolbox is required but not found.');
end
disp('Control System Toolbox found.');

% --- Parameters ---
F_thrust = 27.6e3 / 2;    % Thrust force per engine/actuator [N]
L = 4;                    % Moment arm [m]
I = 9470;                 % Moment of inertia [kg*m^2]

% --- Actuator Model Definitions ---
% ** First-Order Model Parameters **
tau_act = 0.01; % Actuator time constant [s]
G_act_1st = tf(1, [tau_act 1]);

% ** Second-Order Model Parameters **
omega_n = 850;           % Natural frequency [rad/s] % <<< Can adjust this for testing (e.g., 250, 850, 1000)
zeta = 0.707;            % Damping ratio
max_deflection = 0.069813; % Maximum deflection [rad]
min_deflection = -0.069813; % Minimum deflection [rad]
rate_limit = 2.618;      % Rate limit [rad/s]
G_act_2nd = tf(omega_n^2, [1, 2*zeta*omega_n, omega_n^2]);

% --- Select Actuator Model and Tuning Parameters Based on Switch ---
if actuator_model_type == 1
    G_act = G_act_1st;
    model_name = '1st Order';
    % Tuning parameters suitable for 1st order model
    target_bw_inner_hint = 7.5; % [rad/s]
    target_pm_inner = 45; % [degrees]
    target_bw_outer_hint = 0.5; % [rad/s]
    target_pm_outer = 60; % [degrees]
    disp('*** Tuning for 1st Order Actuator Model ***');
    fprintf(' - Time Constant (tau): %.3f s\n', tau_act);

elseif actuator_model_type == 2
    G_act = G_act_2nd;
    model_name = '2nd Order';
    % Tuning parameters suitable for 2nd order model (robust, high PM)
    target_bw_inner_hint = 3.0; % [rad/s]
    target_pm_inner = 60; % [degrees]
    target_bw_outer_hint = 0.3; % [rad/s]
    target_pm_outer = 65; % [degrees]
    disp('*** Tuning for 2nd Order Actuator Model ***');
    fprintf(' - Natural Frequency: %.1f rad/s\n', omega_n);
    fprintf(' - Damping Ratio: %.3f\n', zeta);
    % Only display limits if using 2nd order model
    fprintf(' - Deflection Limits: ±%.4f rad\n', max_deflection);
    fprintf(' - Rate Limit: %.3f rad/s\n', rate_limit);
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
    figure_name_prefix = sprintf('%s Actuator (omega_n=%.0f)', model_name, omega_n); % More descriptive prefix

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
% This section is kept outside the plotting switch as it's informational text
fprintf('\n--- Actuator Limits Note ---\n');
if actuator_model_type == 2
    fprintf('The nonlinear second-order actuator has constraints (not included in linear tuning):\n');
    fprintf(' - Deflection limits: ±%.4f rad (±%.1f deg)\n', max_deflection, max_deflection*180/pi);
    fprintf(' - Rate limit: %.3f rad/s (%.1f deg/s)\n', rate_limit, rate_limit*180/pi);
else
    fprintf('The first-order model used for tuning does not include physical limits.\n');
end
fprintf('For simulation testing (especially with nonlinear actuator):\n');
fprintf(' 1. Ensure anti-windup is enabled in the PID implementation (Simulink blocks).\n');
fprintf(' 2. Monitor actuator commands and outputs to check for saturation.\n');
fprintf(' 3. Use the gains generated by this script (tuned for %s model).\n', model_name);

% --- GAINS FOR SIMULATION (Always Displayed) ---
fprintf('\n--- GAINS FOR SIMULATION ---\n');
fprintf('Inner Kp: %.6f\n', inner_Kp);
fprintf('Inner Ki: %.6f\n', inner_Ki);
fprintf('Inner Kd: %.6f\n', inner_Kd);
fprintf('Outer Kp: %.6f\n', outer_Kp);
fprintf('Outer Ki: %.6f\n', outer_Ki);
fprintf('Outer Kd: %.6f\n', outer_Kd); % This will be 0 for PI controller
fprintf('---------------------------\n');

