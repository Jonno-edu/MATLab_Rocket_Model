%% Rocket Pitch Control Design & Analysis



%% --- 1. System Parameters & Constraints ---
% Vehicle physical properties
T = 27607;        % Thrust (N)
% l_CG = 5.549;     % Nozzle to CG distance (m)
% I_y = 21545.917;  % Pitch moment of inertia (kg*m^2)

l_CG = 5.422;     % Nozzle to CG distance (m)
I_y = 21300;      % Pitch moment of inertia (kg*m^2)

% Actuator hardware model
omega_act = 62;     % Actuator natural frequency (rad/s)
zeta_act = 0.505;   % Actuator damping ratio

%% --- 2. Plant Definition ---
% Define the open-loop dynamics of the rocket pitch rate.

% Aerodynamic model
%pitching_moment_derivative = 5500; % Aerodynamic instability (Nm/rad)
pitching_moment_derivative = 0;
aero_gain = pitching_moment_derivative / I_y;

% Control authority
k_plant = T * l_CG / I_y;

% Create transfer function models
plant_with_aero = tf([k_plant 0], [1 0 -aero_gain]); % Rocket pitch rate dynamics
actuator = tf([omega_act^2], [1 2*zeta_act*omega_act omega_act^2]);

% Combine models to create the full open-loop plant for the inner loop
plant_inner_open_loop = series(actuator, plant_with_aero);

%% --- 3. Inner Loop (Pitch Rate) Controller Design ---
% Design a fast PID controller with a derivative filter to stabilize the pitch rate.
fprintf('--- Inner Loop (Pitch Rate) Design ---\n');

% Design Target:of actuator bandwidth
bw_inner = omega_act / 15;
fprintf('Target Inner Loop Bandwidth: %.2f rad/s\n', bw_inner);

% CORRECTED: Tune a PID controller WITH a derivative filter ('pidf')
opts_inner = pidtuneOptions('PhaseMargin', 60);
[C_inner, ~] = pidtune(plant_inner_open_loop, 'pidf', bw_inner, opts_inner);

% Create the closed-loop inner system
sys_inner_cl = feedback(series(C_inner, plant_inner_open_loop), 1);

% CORRECTED: Calculate the filter coefficient N from the filter time constant Tf
filter_coefficient_N = 1 / C_inner.Tf;

% Display the controller gains and the CRITICAL filter coefficient
disp('Tuned Inner Loop PID Controller (C_inner):');
disp(C_inner);
fprintf('CRITICAL ---> Derivative Filter Coefficient (N): %.2f\n', filter_coefficient_N);



%% --- 4. Outer Loop (Pitch Angle) Controller Design ---
fprintf('\n--- Outer Loop (Pitch Angle) Design ---\n');
bw_outer = bw_inner / 3;
fprintf('Target Outer Loop Bandwidth: %.2f rad/s\n', bw_outer);
plant_outer_open_loop = series(sys_inner_cl, tf(1,[1 0]));
opts_outer = pidtuneOptions('PhaseMargin', 60);
[C_outer, ~] = pidtune(plant_outer_open_loop, 'pi', bw_outer, opts_outer);
sys_outer_cl = minreal(feedback(series(C_outer, plant_outer_open_loop), 1));
disp('Tuned Outer Loop P Controller (C_outer):');
disp(C_outer);



fprintf('\n--- Definitive Stability Check ---\n');
poles_final_system = pole(sys_outer_cl);
if all(real(poles_final_system) < 0)
    fprintf('SUCCESS: The final closed-loop system is STABLE.\n');
else
    fprintf('FAILURE: The final closed-loop system is UNSTABLE.\n');
end


%% Convert to Discrete Controllers
Ts_inner = 1/200; % Sample time, in seconds
Ts_outer = Ts_inner/5;

C_inner_d = c2d(C_inner, Ts_inner, 'tustin') % Discrete inner PID
C_outer_d = c2d(C_outer, Ts_outer, 'tustin') % Discrete outer PI


%% --- 3.5. Export Inner PID Controller Parameters to Workspace ---
C_inner_Kp = C_inner.Kp;
C_inner_Ki = C_inner.Ki;
C_inner_Kd = C_inner.Kd;
C_inner_Tf = C_inner.Tf;
C_inner_N  = 1 / C_inner_Tf;
assignin('base', 'C_inner_Kp', C_inner_Kp);
assignin('base', 'C_inner_Ki', C_inner_Ki);
assignin('base', 'C_inner_Kd', C_inner_Kd);
assignin('base', 'C_inner_Tf', C_inner_Tf);
assignin('base', 'C_inner_N',  C_inner_N);

%% --- 4.5. Export Outer PI Controller Parameters to Workspace ---
C_outer_Kp = C_outer.Kp;
C_outer_Ki = C_outer.Ki;
C_outer_Kd = C_outer.Kd;
assignin('base', 'C_outer_Kp', C_outer_Kp);
assignin('base', 'C_outer_Ki', C_outer_Ki);
assignin('base', 'C_outer_Kd', C_outer_Kd);


%% Discrete Controller Parameters to Workspace
% Inner Pitch rate
C_inner_Ts = Ts_inner;
C_inner_d_Kp = C_inner_d.Kp;
C_inner_d_Ki = C_inner_d.Ki;
C_inner_d_Kd = C_inner_d.Kd;
C_inner_d_Tf = C_inner_d.Tf;
C_inner_d_N  = 1 / C_inner_Tf;
assignin('base', 'C_inner_Ts', C_inner_Ts);
assignin('base', 'C_inner_d_Kp', C_inner_d_Kp);
assignin('base', 'C_inner_d_Ki', C_inner_d_Ki);
assignin('base', 'C_inner_d_Kd', C_inner_d_Kd);
assignin('base', 'C_inner_d_Tf', C_inner_d_Tf);
assignin('base', 'C_inner_d_N',  C_inner_d_N);

%Outer Pitch rate
C_outer_Ts = Ts_outer;
C_outer_d_Kp = C_outer_d.Kp;
C_outer_d_Ki = C_outer_d.Ki;
C_outer_d_Kd = C_outer_d.Kd;
assignin('base', 'Ts_outer', Ts_outer);
assignin('base', 'C_outer_d_Kp', C_outer_d_Kp);
assignin('base', 'C_outer_d_Ki', C_outer_d_Ki);
assignin('base', 'C_outer_d_Kd', C_outer_d_Kd);