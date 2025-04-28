function InitialOut = ControlSystemDesign_func(InitialIn)
%ControlSystemDesign_func Designs PID controllers for rocket pitch control.
%   Takes an Initial structure as input, performs PID tuning based on
%   hardcoded parameters and an identified 2nd order actuator model,
%   and returns the Initial structure updated with control gains.
%
%   Inputs:
%       InitialIn: Structure possibly containing parameters (though currently unused).
%                  Expected to be passed through and returned with added gains.
%   Outputs:
%       InitialOut: The input structure updated with a 'Control' sub-structure
%                   containing fields: inner_Kp, inner_Ki, inner_Kd,
%                   outer_Kp, outer_Ki, outer_Kd.

% --- Configuration ---\
% Using identified 2nd-order actuator model ONLY for this function version
actuator_model_type = 2;
enable_plots = false; % Disable plotting within this function

% --- Check for Control System Toolbox ---\
if isempty(ver('control'))
    error('Control System Toolbox is required but not found.');
end
% disp('Control System Toolbox found.'); % Suppress verbose output

% --- Parameters (Hardcoded based on original script) ---\
F_thrust = 27.6e3 / 1;    % Thrust force per engine/actuator [N]
L = 4;                    % Moment arm [m]
I = 9470;                 % Moment of inertia [kg*m^2] - Consider passing if variable

% --- Actuator Model (Identified 2nd Order) ---\
omega_n = 17.57;           % Natural frequency [rad/s] from identification
zeta = 0.77;               % Damping ratio from identification
max_deflection = 12.5 * pi/180;    % Maximum deflection [rad]
min_deflection = -12.5 * pi/180;   % Minimum deflection [rad]
rate_limit = 75 * pi/180;          % Rate limit [rad/s]

% Identified transfer function
G_act_2nd = tf([0.2133 6824], [1 272 3.091e04]);
G_act = G_act_2nd; % Select the 2nd order model
model_name = 'Identified 2nd Order';

% Tuning parameters adjusted for identified model
target_bw_inner_hint = 2; % [rad/s]
target_pm_inner = 60; % [degrees]
target_bw_outer_hint = 1; % [rad/s]
target_pm_outer = 65; % [degrees]

fprintf('*** Tuning Control System using %s Actuator Model ***\\n', model_name);
% fprintf(' - Natural Frequency: %.2f rad/s (%.2f Hz)\\n', omega_n, omega_n/(2*pi));
% fprintf(' - Damping Ratio: %.3f\\n', zeta);
% fprintf(' - Deflection Limits: ±%.2f rad (±%.2f deg)\\n', max_deflection, max_deflection*180/pi);
% fprintf(' - Rate Limit: %.3f rad/s (%.2f deg/s)\\n', rate_limit, rate_limit*180/pi);

% --- Plant Modeling ---\
s = tf('s');
G_pitchrate = (F_thrust * L / I) / s; % Pitch Dynamics (Torque -> Pitch Rate)
inner_plant_model = G_act * G_pitchrate; % Combined Inner Loop Plant
% fprintf('\\nInner Loop Plant Model:\\n');
% disp(inner_plant_model);

% --- Inner Controller Design (Rate Loop) ---\
fprintf('Tuning Inner Loop PID Controller...\\n');
try
    options_inner = pidtuneOptions('PhaseMargin', target_pm_inner);
    inner_controller = pidtune(inner_plant_model, 'PID', target_bw_inner_hint, options_inner);
catch ME
    error('Failed to tune inner loop controller: %s', ME.message);
end
% fprintf('Inner Loop PID Controller:\\n');
% disp(inner_controller);

% ** Extract Inner Controller Gains **
inner_Kp = inner_controller.Kp;
inner_Ki = inner_controller.Ki;
inner_Kd = inner_controller.Kd;
% fprintf('   Extracted Inner Kp: %.6f\\n', inner_Kp);
% fprintf('   Extracted Inner Ki: %.6f\\n', inner_Ki);
% fprintf('   Extracted Inner Kd: %.6f\\n', inner_Kd);

% --- Inner Loop Analysis (Margins only) ---\
inner_open_loop = inner_controller * inner_plant_model;
inner_loop_closed = feedback(inner_open_loop, 1);
[Gm_inner, Pm_inner, ~, ~] = margin(inner_open_loop);
fprintf('   Inner Loop Margins: Gm=%.2f dB, Pm=%.2f deg\\n', 20*log10(Gm_inner), Pm_inner);

% --- Outer Plant Definition (Angle Loop) ---\
outer_plant = inner_loop_closed * (1 / s);
% fprintf('\\nOuter Loop Plant TF:\\n');
% disp(outer_plant);

% --- Outer Controller Design (Angle Loop) ---\
fprintf('Tuning Outer Loop PI Controller...\\n');
try
    options_outer = pidtuneOptions('PhaseMargin', target_pm_outer);
    angle_controller = pidtune(outer_plant, 'PI', target_bw_outer_hint, options_outer);
catch ME
    error('Failed to tune outer loop controller: %s', ME.message);
end
% fprintf('Outer Loop PI Controller:\\n');
% disp(angle_controller);

% ** Extract Outer Controller Gains **
outer_Kp = angle_controller.Kp;
outer_Ki = angle_controller.Ki;
outer_Kd = 0; % Explicitly set to 0 for PI controller
% fprintf('   Extracted Outer Kp: %.6f\\n', outer_Kp);
% fprintf('   Extracted Outer Ki: %.6f\\n', outer_Ki);
% fprintf('   Extracted Outer Kd: %.6f\\n', outer_Kd);

% --- Outer Loop Analysis (Margins only) ---\
outer_open_loop = angle_controller * outer_plant;
outer_cl = feedback(outer_open_loop, 1);
[Gm, Pm, ~, ~] = margin(outer_open_loop);
fprintf('   Outer Loop Margins: Gm=%.2f dB, Pm=%.2f deg\\n', 20*log10(Gm), Pm);

% --- Package Gains into Output Structure ---
InitialOut = InitialIn; % Start with the input structure
InitialOut.Control.inner_Kp = inner_Kp;
InitialOut.Control.inner_Ki = inner_Ki;
InitialOut.Control.inner_Kd = inner_Kd;
InitialOut.Control.outer_Kp = outer_Kp;
InitialOut.Control.outer_Ki = outer_Ki;
InitialOut.Control.outer_Kd = outer_Kd; % Will be 0

fprintf('Control gains added to Initial structure.\\n');
fprintf('--- Control Design Function Complete ---\\n');

end % End of function ControlSystemDesign_func
