% LQR Controller Design for a Single Flight Point
% This script prompts the user for a specific time, loads flight data,
% extracts the vehicle parameters at that instant, and designs the
% LQR controller using the correct state-space formulation.
%clear; clc; close all;

%% 1. User Input and Simulation Data Loading
% --- SET THE DESIRED TIME FOR ANALYSIS HERE ---
target_time = 0.5; 

unit_test = true;


% Load simulation data
filename = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'output', 'simOut_heavy_benchmark.mat'); % Updated for new structure
try
    load(filename);
    fprintf('Successfully loaded flight data from %s.\n', filename);
catch
    error('File not found: %s. Please check the filename and path.', filename);
end

%% 2. Define Fixed System Parameters & LQR Tuning Weights
% Fixed parameters
S = 0.200296;           % Reference area (m²)
L = 9.542;              % Reference length (m)

% Actuator model (states are [position; rate])
natural_frequency = 62; % rad/s
damping_ratio = 0.5858;
wn = natural_frequency;
zeta = damping_ratio;
A_act = [0 1; -wn^2 -2*zeta*wn];
B_act = [0; wn^2];
C_act = [1 0]; % Actuator output is position

% LQR Tuning Weights
max_alpha       = 1 * pi/180;  % rad
max_q           = 1 * pi/180;% rad/s (pitch rate)
max_theta       = 1 * pi/180;% rad
max_act_pos     = 2 * pi/180;  % rad
max_act_rate    = 10 * pi/180; % rad/s

max_TVC_command = 2 * pi/180;  % rad

% Construct Q and R matrices
Q_diag = [1/max_alpha^2, 1/max_q^2, 1/max_theta^2, 1/max_act_pos^2, 1/max_act_rate^2];
Q = diag(Q_diag);
R = 1/max_TVC_command^2;

%% 3. Extract and Interpolate Flight Data at Target Time
logsout_data = simOut.logsout;

% Define all the signal paths to be extracted
paths = struct();
paths.V = 'airspeed';
paths.rho = 'ENV.AirDensity';
paths.mass = 'mass';
paths.I = 'I';
paths.T = 'engineThrust';
paths.CNa = 'Cnalpha';
paths.CMa = 'Cmalpha';
paths.Cmq = 'cmq';
paths.L_arm = 'CG_X';  % TVC moment arm (CG from tail)
paths.Alt = 'PlantData.alt'; % CORRECTED PATH for altitude

% Extract and interpolate each required parameter at the target time
fprintf('\nExtracting flight parameters at t = %.2f s...\n', target_time);
params = struct();
field_names = fieldnames(paths);
for i = 1:length(field_names)
    signal_name = field_names{i};
    signal_path = paths.(signal_name);

    try
        signal_obj = get_nested_signal(logsout_data, signal_path);
        ts = signal_obj.Values;

        if strcmp(signal_name, 'I')
            % Handle 3D inertia matrix, extracting Iyy
            inertia_data = squeeze(ts.Data(2,2,:));
            params.(signal_name) = interp1(ts.Time, inertia_data, target_time, 'linear', 'extrap');
        else
            params.(signal_name) = interp1(ts.Time, squeeze(ts.Data), target_time, 'linear', 'extrap');
        end
    catch ME
        error('Failed to extract or interpolate signal "%s": %s', signal_path, ME.message);
    end
end

% Display the extracted parameters for verification
fprintf('--- Vehicle State at t = %.2f s ---\n', target_time);
fprintf('Altitude (Alt): %.1f m\n', params.Alt); % Display altitude
fprintf('Mass (m):       %.2f kg\n', params.mass);
fprintf('Velocity (V):   %.2f m/s\n', params.V);
fprintf('Density (rho):  %.4f kg/m^3\n', params.rho);
fprintf('Thrust (T):     %.0f N\n', params.T);
fprintf('Inertia (Iyy):  %.0f kg*m^2\n', params.I);
fprintf('TVC Arm (L_arm):%.3f m\n', params.L_arm);
fprintf('CNa:            %.3f\n', params.CNa);
fprintf('CMa:            %.3f\n', params.CMa);
fprintf('Cmq:            %.3f\n', params.Cmq);
fprintf('------------------------------------\n\n');


%% 4. Main LQR Design for the Single Flight Point
% Recalculate dimensional stability derivatives using the extracted data
q_bar = 0.5 * params.rho * params.V^2;
Z_alpha = (q_bar * S / params.mass) * params.CNa + (params.T / params.mass);
Z_delta = params.T / params.mass;
M_alpha = (q_bar * S * L / params.I) * params.CMa;
M_q = (q_bar * S * L^2) / (2 * params.V * params.I) * params.Cmq;
M_delta = (params.T * params.L_arm) / params.I;

% Define the airframe-only state-space model
% States: [alpha; q; theta]
A_plant = [-Z_alpha/params.V,  1,   0;
            M_alpha,           M_q, 0;
            0,                 1,   0];

B_plant = [-Z_delta/params.V;
            M_delta;
            0];

% Define the combined plant and actuator model for LQR design
% States: [alpha; q; theta; act_pos; act_rate]
A_current = [[A_plant, B_plant*C_act]; [zeros(2,3), A_act]];
B_current = [B_plant*0; B_act]; % Assuming D_act is 0

% Design the LQR controller for this specific flight point
K_lqr = lqr(A_current, B_current, Q, R);

%% 5. Display Final Results for the Target Time
fprintf('--- LQR Controller Design Complete for t = %.2f s ---\n', target_time);
disp('LQR Gain Matrix K:');
disp(K_lqr);

disp('Eigenvalues of the closed-loop system (A - B*K):');
disp(eig(A_current - B_current * K_lqr));
fprintf('Note: All eigenvalues should have negative real parts for stability.\n');


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
