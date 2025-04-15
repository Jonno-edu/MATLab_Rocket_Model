%% 1. Configure and Simulate with Delayed Step Input
mdl = 'actuator_models';
load_system(mdl);

% Step parameters
step_value = 4*pi/180;  % ≈0.0698 radians (4 degrees)
step_delay = 2;         % 2-second delay recommended

% Configure Step block
set_param([mdl '/Step'],...
    'Time', num2str(step_delay),...
    'Before', '0',...
    'After', num2str(step_value));

% Simulation parameters
sim_time = 10;          % Total simulation time
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
Ts = t(2) - t(1);       % Actual sampling time
data = iddata(y, u, Ts);

% Model estimation (2 poles, 1 zero)
np = 2;                 % Number of poles
nz = 1;                 % Number of zeros
sys = tfest(data, np, nz);

%% 4. Validation and Visualization
figure;
compare(data, sys);
grid on;
title('Model vs Actual Response');

%% 5. Display Results
disp('Estimated Transfer Function:');
sys
