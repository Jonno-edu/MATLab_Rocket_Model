%% 1. Configure and Simulate with Delayed Step Input
mdl = 'actuator_models';
load_system(mdl);

% Step parameters
step_time_actual = 1;     % Time when the step occurs (seconds)
initial_value_actual = 0; % Initial value of the step
final_value_actual = 0.035; % Final value of the step (0.035m)

% Configure Step block in the Simulink model
set_param([mdl '/Step'],...
    'Time', num2str(step_time_actual),...
    'Before', num2str(initial_value_actual),...
    'After', num2str(final_value_actual));

% Simulation parameters
sim_time = 10;           % Total simulation time
out = sim(mdl, sim_time);

%% 2. Extract and Prepare Data
% Get actuator output signal
actuator_data_sim = out.get('actuator'); % Changed variable name to avoid conflict
y = actuator_data_sim.Data;
t = actuator_data_sim.Time;

% Create the input signal 'u' that was fed to the system
% This should precisely match the Step block's behavior
u = zeros(size(t));
% Apply initial value before step_time_actual
u(t < step_time_actual) = initial_value_actual;
% Apply final value at and after step_time_actual
u(t >= step_time_actual) = final_value_actual;


%% 3. Transfer Function Estimation
Ts = t(2) - t(1);        % Actual sampling time
data = iddata(y, u, Ts);

% Model estimation (2 poles, 1 zero - adjust if needed)
np = 2;                  % Number of poles
nz = 1;                  % Number of zeros
sys = tfest(data, np, nz);

%% 4. Validation and Visualization
figure;
compare(data, sys);
grid on;
title('Model vs Actual Response');

% Add frequency response visualization
figure;
bode(sys);
grid on;
title('Frequency Response of Servo Actuator');

%% 5. Calculate and Display Key Performance Metrics
% Extract transfer function parameters
[num, den] = tfdata(sys, 'v');

% Check if the identified system is valid before calculating metrics
if ~isempty(den) && den(end) ~= 0 && length(den) >= 3 % Ensure at least a second-order system for wn, zeta
    wn = sqrt(den(end)/den(1)); % For standard form s^2 + 2*zeta*wn*s + wn^2, assuming den(1) might not be 1
    % Adjusting for general form a*s^2 + b*s + c, wn = sqrt(c/a)
    if den(1) ~= 0
        wn = sqrt(den(3)/den(1));
        zeta = (den(2)/den(1))/(2*wn);
    else % if den(1) is zero, it's not a standard second order form this way
        wn = sqrt(den(3)); % Fallback, assumes den(1) was meant to be 1
        zeta = den(2)/(2*wn);
    end
    tr_approx = 1.8/wn;      % Approximate rise time

    fprintf('Servo Actuator Performance Metrics:\n');
    fprintf('Natural Frequency: %.2f rad/s (%.2f Hz)\n', wn, wn/(2*pi));
    fprintf('Damping Ratio: %.2f\n', zeta);
    fprintf('Approximate Rise Time: %.3f seconds\n', tr_approx);
else
    fprintf('Could not reliably calculate performance metrics from the identified system.\n');
    if isempty(den)
        fprintf('Denominator of the transfer function is empty.\n');
    elseif den(end) == 0
        fprintf('The last coefficient of the denominator is zero, wn calculation problematic.\n');
    elseif length(den) < 3
        fprintf('The system order is less than 2, wn and zeta calculation for 2nd order system is not applicable.\n');
    end
end


%% 6. Export Model for Controller Design
% Create state-space model
sys_ss = ss(sys);

% Save model for controller design
save('servo_model_for_control.mat', 'sys', 'sys_ss');

fprintf('\nModel saved for controller design. Use for:\n');
fprintf('1. PID tuning with "pidtune(sys, ''PIDF'')" \n');
fprintf('2. State-space control with "lqr(sys_ss.A, sys_ss.B, Q, R)" \n');
fprintf('3. Robust control with "hinfsyn" or "mixsyn" \n');

disp('Estimated Transfer Function:');
sys
