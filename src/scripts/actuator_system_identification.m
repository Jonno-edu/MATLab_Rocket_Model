%% 1. Configure and Simulate with Delayed Step Input
mdl = 'actuator_models';
load_system(mdl);

% Step parameters
step_value = 50*pi/180;  % 50 degrees in radians
step_delay = 2;          % 2-second delay recommended

% Configure Step block
set_param([mdl '/Step'],...
    'Time', num2str(step_delay),...
    'Before', '0',...
    'After', num2str(step_value));

% Simulation parameters
sim_time = 10;           % Total simulation time
out = sim(mdl, sim_time);

%% 2. Extract and Prepare Data
% Get output signal
actuator_data = out.get('actuator');
y = actuator_data.Data;
t = actuator_data.Time;

% Create input signal matching simulation
u = zeros(size(t));
u(t >= step_delay) = step_value;

%% 3. Transfer Function Estimation
Ts = t(2) - t(1);        % Actual sampling time
data = iddata(y, u, Ts);

% Model estimation (2 poles, 1 zero)
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
wn = sqrt(den(3));       % Natural frequency
zeta = den(2)/(2*wn);    % Damping ratio
tr_approx = 1.8/wn;      % Approximate rise time

fprintf('Servo Actuator Performance Metrics:\n');
fprintf('Natural Frequency: %.2f rad/s (%.2f Hz)\n', wn, wn/(2*pi));
fprintf('Damping Ratio: %.2f\n', zeta);
fprintf('Approximate Rise Time: %.3f seconds\n', tr_approx);

%% 6. Enhanced Servo Model with Physical Constraints
% Create enhanced model with saturation and rate limits
servo_max_angle = 60*pi/180;   % ±60 degrees in radians
servo_max_rate = 300*pi/180;   % 300 deg/s in rad/s

% Create saturation and rate limit blocks within enhanced model
sys_enhanced = sys;            % Start with identified model

% Define servo-to-nozzle conversion function for TVC
L_servo = 0.02;                % 20mm servo arm length
L_nozzle = 0.08;               % 80mm nozzle bell crank length
leverage_ratio = L_servo/L_nozzle;

% Conversion function (to be used in Simulink Function block)
servo2nozzle = @(servo_angle) asin(leverage_ratio * sin(servo_angle));

% Compute max nozzle angle based on servo limits
max_nozzle_angle = servo2nozzle(servo_max_angle);
fprintf('Maximum Nozzle Deflection: %.2f degrees\n', max_nozzle_angle*180/pi);

% Maximum nozzle angular rate based on servo limits
max_rate_factor = leverage_ratio * cos(0);  % Maximum leverage effect (at center position)
max_nozzle_rate = servo_max_rate * max_rate_factor;
fprintf('Maximum Nozzle Rate: %.2f deg/s\n', max_nozzle_rate*180/pi);

%% 7. Simulate Enhanced Model with Step Input
% Time vector for simulation
t_sim = 0:0.001:0.3;      % 300ms simulation with 1kHz sampling

% Step input (directly to enhanced model)
u_sim = zeros(size(t_sim));
u_sim(t_sim >= 0.01) = 50*pi/180;  % 15-degree step at t=10ms

% Simulate enhanced model (with manual saturation and rate limiting)
[y_servo, ~, ~] = lsim(sys, u_sim, t_sim);

% Apply saturation
y_servo = min(servo_max_angle, max(y_servo, -servo_max_angle));

% Apply rate limiting (approximate implementation)
y_servo_rate_limited = zeros(size(y_servo));
y_servo_rate_limited(1) = y_servo(1);
dt = t_sim(2) - t_sim(1);
for i = 2:length(t_sim)
    max_change = servo_max_rate * dt;
    y_servo_rate_limited(i) = y_servo_rate_limited(i-1) + ...
        min(max_change, max(y_servo(i) - y_servo_rate_limited(i-1), -max_change));
end

% Convert servo angle to nozzle angle
y_nozzle = zeros(size(y_servo_rate_limited));
for i = 1:length(y_servo_rate_limited)
    y_nozzle(i) = servo2nozzle(y_servo_rate_limited(i));
end

% Plot results
figure;
subplot(2,1,1);
plot(t_sim, u_sim*180/pi, 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, y_servo_rate_limited*180/pi, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Command', 'Servo Response');
title('Servo Response with Rate and Position Limits');

subplot(2,1,2);
plot(t_sim, y_nozzle*180/pi, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Nozzle Response via Pushrod Linkage');

%% 8. Export Model for Controller Design
% Create state-space model
sys_ss = ss(sys);

% Save model for controller design
save('servo_model_for_control.mat', 'sys', 'sys_ss', 'servo_max_angle', ...
     'servo_max_rate', 'leverage_ratio', 'servo2nozzle');

fprintf('\nModel saved for controller design. Use for:\n');
fprintf('1. PID tuning with "pidtune(sys, ''PIDF'')" \n');
fprintf('2. State-space control with "lqr(sys_ss.A, sys_ss.B, Q, R)" \n');
fprintf('3. Robust control with "hinfsyn" or "mixsyn" \n');

disp('Estimated Transfer Function:');
sys
