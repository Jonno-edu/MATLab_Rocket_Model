% design_lqr_at_time.m
% This script designs an LQR controller for a single flight condition
% and ensures all matrices are of the correct 'double' data type for Simulink.

clear;
clc;
close all;

%% --- Configuration ---
% Define the point in time (in seconds) for which to design the controller.
query_time = 40; % Example: Design for t=10s

%% 1. Setup and Load Simulation Data
filename = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/output_data/simOut_heavy_benchmark.mat';
try
    load(filename);
    fprintf('Successfully loaded flight data from %s.\n', filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

%% 2. Extract and Interpolate Flight Data at query_time
fprintf('\nFetching flight data for t = %.1f s...\n', query_time);
logsout_data = simOut.logsout;
paths = struct();
paths.V = 'airspeed';
paths.rho = 'ENV.AirDensity';
paths.mass = 'mass';
paths.I = 'I';
paths.T = 'engineThrust';
paths.CNa = 'Cnalpha';
paths.CMa = 'Cmalpha';
paths.Cmq = 'cmq';
paths.L_arm = 'CG_X';

interp_data = struct();
field_names = fieldnames(paths);
for i = 1:length(field_names)
    signal_name = field_names{i};
    signal_path = paths.(signal_name);
    try
        signal_obj = get_nested_signal(logsout_data, signal_path);
        ts = signal_obj.Values;
        if strcmp(signal_name, 'I')
            inertia_data = squeeze(ts.Data(2,2,:));
            interp_data.(signal_name) = interp1(ts.Time, inertia_data, query_time, 'linear', 'extrap');
        else
            interp_data.(signal_name) = interp1(ts.Time, squeeze(ts.Data), query_time, 'linear', 'extrap');
        end
    catch ME
        error('Failed to extract or interpolate signal "%s": %s', signal_path, ME.message);
    end
end
fprintf('Data fetch complete.\n');

%% 3. Assign Fetched Parameters
% Fixed parameters
S = 0.200296;
L = 9.542;

% Assign interpolated physical parameters
m = interp_data.mass;
rho = interp_data.rho;
V = interp_data.V;
I = interp_data.I;
T = interp_data.T;
L_arm = interp_data.L_arm;

% Assign interpolated aerodynamic coefficients and ensure correct signs
CNa = interp_data.CNa;
Cma = abs(interp_data.CMa);
Cmq = -abs(interp_data.Cmq);

% Robustness check for velocity at t=0
if V < 1.0; V = 1.0; end

%% 4. Dimensional Stability Derivatives
q_bar = 0.5 * rho * V^2;
Z_alpha = (q_bar * S / m) * CNa;
Z_delta = T / m;
M_alpha = (q_bar * S * L / I) * Cma;
M_q = (q_bar * S * L^2) / (2 * V * I) * Cmq;
M_delta = (T * L_arm) / I;

%% 5. Rocket Airframe State-Space Model (3 states)
A_plant_raw = [-Z_alpha/V,  1,   0;
               M_alpha,    M_q, 0;
               0,          1,   0];

B_plant_raw = [-Z_delta/V;
               M_delta;
               0];

% --- DATA TYPE CORRECTION ---
% Ensure all matrices are double-precision, which Simulink expects.
% This prevents errors if any upstream calculation produces a single.
A_plant = double(A_plant_raw);
B_plant = double(B_plant_raw);
C_plant = double(eye(3));
D_plant = double(zeros(3,1));

sys_plant = ss(A_plant, B_plant, C_plant, D_plant);
sys_plant.StateName = {'alpha', 'q', 'theta'};

%% 6. Actuator State-Space Model (2 states)
natural_frequency = 62;
damping_ratio = 0.5858;
num_actuator = natural_frequency^2;
den_actuator = [1, 2*damping_ratio*natural_frequency, natural_frequency^2];
G_actuator = tf(num_actuator, den_actuator);

sys_actuator = ss(G_actuator);
sys_actuator.StateName = {'act_pos', 'act_rate'};

%% 7. Combine Models
sys_open_loop = series(sys_actuator, sys_plant);
A = sys_open_loop.A;
B = sys_open_loop.B;

%% 8. LQR Controller Design
max_devs = struct();
max_devs.act_pos     = 3 * pi/180;
max_devs.act_rate    = 50 * pi/180;
max_devs.alpha       = 0.5 * pi/180;
max_devs.q           = 0.2 * pi/180;
max_devs.theta       = 10 * pi/180;
max_TVC_command      = 4 * pi/180;

actual_state_order = sys_open_loop.StateName;
num_states = length(actual_state_order);
Q_diag = zeros(1, num_states);
for i = 1:num_states
    state_name = actual_state_order{i};
    Q_diag(i) = 1 / max_devs.(state_name)^2;
end
Q = diag(Q_diag);
R = 1/max_TVC_command^2;
K_lqr = lqr(A, B, Q, R);

%% --- Final Results ---
fprintf('\n--- LQR Design Complete for t = %.1f s ---\n', query_time);
disp('LQR Gain Matrix K:');
disp(K_lqr);
disp('Closed-loop Eigenvalues:');
disp(eig(A - B*K_lqr));


%% Helper Function to Extract Nested Signals from logsout
function signal_obj = get_nested_signal(logsout_data, path_str)
    path_parts = strsplit(path_str, '.');
    try
        current_obj = logsout_data.getElement(path_parts{1});
        if length(path_parts) > 1
            current_struct = current_obj.Values;
            for k = 2:length(path_parts)
                current_struct = current_struct.(path_parts{k});
            end
            signal_obj.Values = current_struct;
        else
            signal_obj = current_obj;
        end
    catch ME
        rethrow(ME);
    end
end
