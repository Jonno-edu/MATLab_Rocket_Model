% Gain Scheduling LQR – Simplified for Feedforward Architecture
% clear;
% clc;
% close all;

%% 1. Load Simulation Data
filename = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/output_data/simOut_heavy_benchmark.mat';
try
    load(filename);
    fprintf('Successfully loaded flight data from %s.\n', filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

%% 2. Fixed System & Controller Parameters
S = 0.200296;
L = 9.542;
natural_frequency = 62;
damping_ratio = 0.5858;
wn = natural_frequency;
zeta = damping_ratio;
A_act = [0 1; -wn^2 -2*zeta*wn];
B_act = [0; wn^2];
C_act = [1 0];

% --- Single, Relaxed Tuning Profile for LQR Stabilizer ---
max_devs = struct();
max_devs.alpha    = 100 * pi/180; % Typically not penalized heavily
max_devs.q        = 6 * pi/180;   % Relaxed pitch rate penalty
max_devs.theta    = 1 * pi/180;  % Very relaxed attitude penalty
max_devs.act_pos  = 5 * pi/180;  % Relaxed actuator position penalty
max_devs.act_rate = 1 * pi/180;   % Relaxed actuator rate penalty

max_TVC_command = 4 * pi/180; % A reasonable control authority

Q_diag = [1/max_devs.alpha^2, 1/max_devs.q^2, 1/max_devs.theta^2, ...
          1/max_devs.act_pos^2, 1/max_devs.act_rate^2];
Q = diag(Q_diag);
R = 1/max_TVC_command^2;
N = zeros(5, 1);

%% 3. Time Vector
max_design_time = 98;
full_time_vector = (0:1:145)';
num_points = length(full_time_vector);

%% 4. Extract & Interpolate Time-Varying Flight Data
logsout_data = simOut.logsout;
paths = struct();
paths.V      = 'airspeed';
paths.rho    = 'ENV.AirDensity';
paths.mass   = 'mass';
paths.I      = 'I';
paths.T      = 'engineThrust';
paths.CNa    = 'Cnalpha';
paths.CMa    = 'Cmalpha';
paths.Cmq    = 'cmq';
paths.L_arm  = 'CG_X';

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
            interp_data.(signal_name) = interp1(ts.Time, inertia_data, full_time_vector, 'linear', 'extrap');
        else
            interp_data.(signal_name) = interp1(ts.Time, squeeze(ts.Data), full_time_vector, 'linear', 'extrap');
        end
    catch ME
        error('Failed to extract or interpolate signal "%s": %s', signal_path, ME.message);
    end
end
fprintf('Interpolation complete.\n');

%% 5. Main Loop: Design Controller with Fixed Tuning
K_gains = zeros(num_points, 5);

for i = 2:num_points
    t = full_time_vector(i);

    % --- Plant Dynamics Calculation (as before) ---
    m = interp_data.mass(i); V = interp_data.V(i); rho = interp_data.rho(i);
    T = interp_data.T(i); I = interp_data.I(i); CNa_current = interp_data.CNa(i);
    CMa_current = abs(interp_data.CMa(i)); Cmq_current = -abs(interp_data.Cmq(i));
    L_arm_current = interp_data.L_arm(i);

    if T < 1.0 % Handle burnout
        for j = i:num_points
            K_gains(j, :) = K_gains(i-1, :);
        end
        break;
    end

    q_bar = 0.5 * rho * V^2;
    Z_alpha = (q_bar * S / m) * CNa_current;
    Z_delta = T / m;
    M_alpha = (q_bar * S * L / I) * CMa_current;
    M_q = (q_bar * S * L^2) / (2 * V * I) * Cmq_current;
    M_delta = (T * L_arm_current) / I;

    A_plant = [-Z_alpha/V, 1, 0; M_alpha, M_q, 0; 0, 1, 0];
    B_plant = [-Z_delta/V; M_delta; 0];
    A_current = [[A_plant, B_plant*C_act]; [zeros(2,3), A_act]];
    B_current = [[zeros(3,1)]; B_act];

    % The Q and R matrices are now constant, only A and B change
    K_gains(i, :) = lqr(A_current, B_current, Q, R, N);
end

K_gains(1, :) = K_gains(2, :); % Handle t=0

%% 6. Export Results for Simulink
assignin('base', 'time_vector', full_time_vector);
assignin('base', 'K_gains', K_gains);

%% 7. Plotting Controller Gains
figure;
hold on;
plot(full_time_vector, K_gains(:,1), 'LineWidth', 1.5, 'DisplayName', 'K1 (alpha)');
plot(full_time_vector, K_gains(:,2), 'LineWidth', 1.5, 'DisplayName', 'K2 (q)');
plot(full_time_vector, K_gains(:,3), 'LineWidth', 1.5, 'DisplayName', 'K3 (theta)');
plot(full_time_vector, K_gains(:,4), 'LineWidth', 1.5, 'DisplayName', 'K4 (act\_pos)');
plot(full_time_vector, K_gains(:,5), 'LineWidth', 1.5, 'DisplayName', 'K5 (act\_rate)');
hold off;
title('Gain-Scheduled LQR Gains vs. Time');
xlabel('Time (s)');
ylabel('Gain Value');
legend('show', 'Location', 'best');
grid on;
xlim([0, full_time_vector(end)]);

%% Helper Function
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
