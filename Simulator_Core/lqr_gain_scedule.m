% Gain Scheduling LQR Controller Design (Final Version with Debugging)
% This script handles the t=0 singularity by using the gains from the
% first valid flight condition and includes periodic debugging printouts.

%% 1. Setup and Load Simulation Data
filename = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/output_data/simOut_heavy_benchmark.mat';
try
    load(filename);
    fprintf('Successfully loaded flight data from %s.\n', filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

%% 2. Define Fixed System Parameters & LQR Tuning Weights
S = 0.200296;
L = 9.542;
natural_frequency = 62;
damping_ratio = 0.5858;
wn = natural_frequency;
zeta = damping_ratio;
A_act = [0 1; -wn^2 -2*zeta*wn];
B_act = [0; wn^2];
C_act = [1 0];

% --- TUNING CORRECTION ---
% The previous weights were too aggressive, causing high gains and oscillations.
% These have been relaxed to produce a smoother, more robust controller.
max_devs = struct();
max_devs.alpha       = 20 * pi/180;     % Allow slightly more AoA error
max_devs.q           = 0.5 * pi/180;    % Allow more pitch rate error
max_devs.theta       = 1 * pi/180;     % Relax theta penalty
max_devs.act_pos     = 2 * pi/180;     % Actuator position penalty
max_devs.act_rate    = 1 * pi/180;    % Relax the actuator rate penalty
max_TVC_command      = 0.8 * pi/180;     % Drastically relax the control effort penalty

Q_diag = [1/max_devs.alpha^2, 1/max_devs.q^2, 1/max_devs.theta^2, 1/max_devs.act_pos^2, 1/max_devs.act_rate^2];
Q = diag(Q_diag);
R = 1/max_TVC_command^2;

%% 3. Define Time Vector for Scheduling
time_vector = (0:1:145)';

%% 4. Extract and Interpolate ALL Time-Varying Flight Data
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

fprintf('Interpolating all required flight data...\n');
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
            interp_data.(signal_name) = interp1(ts.Time, inertia_data, time_vector, 'linear', 'extrap');
        else
            interp_data.(signal_name) = interp1(ts.Time, squeeze(ts.Data), time_vector, 'linear', 'extrap');
        end
    catch ME
        error('Failed to extract or interpolate signal "%s": %s', signal_path, ME.message);
    end
end
fprintf('Data interpolation complete.\n\n');

%% 5. Main Loop: Design Controller for Each Time Point
num_points = length(time_vector);
K_gains = zeros(num_points, 5);
fprintf('Designing gain-scheduled LQR controllers for t=0s to t=%ds...\n', time_vector(end));

% Start the loop from the SECOND point (i=2) to avoid V=0 at t=0.
for i = 2:num_points
    t = time_vector(i);
    m = interp_data.mass(i);
    V = interp_data.V(i);
    rho = interp_data.rho(i);
    T = interp_data.T(i);
    I = interp_data.I(i);
    CNa_current = interp_data.CNa(i);
    CMa_current = abs(interp_data.CMa(i));
    Cmq_current = -abs(interp_data.Cmq(i));
    L_arm_current = interp_data.L_arm(i);
    
    % --- DEBUGGING PRINTOUT ---
    % Periodically print the interpolated flight parameters to the console.
    % The loop index `i` corresponds to t+1, so we check mod(t, 10).
    if mod(t, 30) == 0 && t > 0
        fprintf('--------------------------------------------------------------------------------------------------\n');
        fprintf('DEBUG @ t = %.1f s:\n', t);
        fprintf('  Mass  = %.2f kg | Velocity = %.2f m/s | Density = %.4f kg/m^3 | Thrust = %.2f N\n', m, V, rho, T);
        fprintf('  Inertia = %.2f kg*m^2 | CNa = %.4f | CMa = %.4f | Cmq = %.4f | TVC Arm = %.4f m\n', I, CNa_current, CMa_current, Cmq_current, L_arm_current);
        fprintf('--------------------------------------------------------------------------------------------------\n');
    end
    
    % Check for engine burnout (uncontrollable case)
    if T < 1.0
        fprintf('Thrust is zero at t=%.1f s. Using last valid gains for remainder of flight.\n', t);
        for j = i:num_points
            K_gains(j, :) = K_gains(i-1, :);
        end
        break; % Exit the loop
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
    
    K_gains(i, :) = lqr(A_current, B_current, Q, R);
end

% Handle the t=0 case by copying the gains from the first valid point.
K_gains(1, :) = K_gains(2, :);
fprintf('Gains for t=0 have been set to match the gains from t=%.1f s.\n', time_vector(2));


%% 6. Export Results to Base Workspace for Simulink
fprintf('\n--- Gain Scheduling Design Complete ---\n\n');
assignin('base', 'time_vector', time_vector);
assignin('base', 'K_gains', K_gains);
fprintf('Variables are ready for Simulink.\n');

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
