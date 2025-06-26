function generate_blender_csv(simOut)
% Generate a CSV for Blender animation from simOut

csv_filename = 'trajectory_for_blender_60fps.csv';
output_folder = fullfile(fileparts(mfilename('fullpath')), 'output_data');
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end
csv_fullpath = fullfile(output_folder, csv_filename);

logsout = simOut.logsout;

% Get data from logsout
plant_data = logsout.getElement('PlantData').Values;

% Get main time vector
t_main = plant_data.body_angles.theta.Time(:);

% Position data - check if available
try
    pos_x = plant_data.Xe.Data(:, 1);
    pos_y = plant_data.Xe.Data(:, 2);
    pos_z = -1 * plant_data.Xe.Data(:, 3);
    t_pos = t_main; % Assume position uses same time as plant_data
catch
    warning('Position data (Xe) not found - using zeros');
    pos_x = zeros(size(t_main));
    pos_y = zeros(size(t_main));
    pos_z = zeros(size(t_main));
    t_pos = t_main;
end

% Pitch angle
pitch_rad = plant_data.body_angles.theta.Data(:);
t_pitch = t_main;

% Center of gravity data - check if available
try
    cg_body_x = plant_data.CG.Data(:);
    t_cg = t_main; % Assume CG uses same time as plant_data
catch
    warning('CG data not found - using zeros');
    cg_body_x = zeros(size(t_main));
    t_cg = t_main;
end

% Nozzle angle - get its own time vector and fix length mismatch
nozzle_data = logsout.getElement("Y_nozzle_angle").Values;
nozzle_angle_rad = nozzle_data.Data(:);
t_nozzle = nozzle_data.Time(:);

% Fix length mismatch for nozzle data
min_len_nozzle = min(length(t_nozzle), length(nozzle_angle_rad));
t_nozzle = t_nozzle(1:min_len_nozzle);
nozzle_angle_rad = nozzle_angle_rad(1:min_len_nozzle);

% Thrust data - get its own time vector and fix length mismatch
try
    thrust_data = logsout.getElement('engineThrust').Values;
    thrust = thrust_data.Data(:);
    t_thrust = thrust_data.Time(:);
    
    % Fix length mismatch for thrust data
    min_len_thrust = min(length(t_thrust), length(thrust));
    t_thrust = t_thrust(1:min_len_thrust);
    thrust = thrust(1:min_len_thrust);
    
    max_thrust = evalin('base', 'Actuators.Engine.MaxThrust');
    if max_thrust > 0
        thrust_normalized = thrust / max_thrust;
    else
        thrust_normalized = zeros(size(thrust));
    end
catch
    warning('Engine thrust data not found - using zeros');
    thrust_normalized = zeros(size(t_main));
    t_thrust = t_main;
end

% Create a common time base for all signals
t_start = max([t_main(1), t_pos(1), t_pitch(1), t_cg(1), t_nozzle(1), t_thrust(1)]);
t_end = min([t_main(end), t_pos(end), t_pitch(end), t_cg(end), t_nozzle(end), t_thrust(end)]);

% Resample all signals to 60 Hz (60 fps)
fps = 60;
t_uniform = (t_start:1/fps:t_end)';

% Interpolate all signals to uniform timebase using their respective time vectors
pos_x_interp = interp1(t_pos, pos_x, t_uniform, 'linear');
pos_y_interp = interp1(t_pos, pos_y, t_uniform, 'linear');
pos_z_interp = interp1(t_pos, pos_z, t_uniform, 'linear');
pitch_rad_interp = interp1(t_pitch, pitch_rad, t_uniform, 'linear');
cg_body_x_interp = interp1(t_cg, cg_body_x, t_uniform, 'linear');
nozzle_angle_rad_interp = interp1(t_nozzle, nozzle_angle_rad, t_uniform, 'linear');
thrust_normalized_interp = interp1(t_thrust, thrust_normalized, t_uniform, 'linear');

% Create zero arrays for Y and Z CG components
cg_body_y_interp = zeros(size(t_uniform));
cg_body_z_interp = zeros(size(t_uniform));

% Create table
N = length(t_uniform);
T = table(t_uniform, pos_x_interp, pos_y_interp, pos_z_interp, pitch_rad_interp, ...
    cg_body_x_interp, cg_body_y_interp, cg_body_z_interp, ...
    nozzle_angle_rad_interp, thrust_normalized_interp, ...
    'VariableNames', {'time_s','pos_x_m','pos_y_m','pos_z_m','pitch_rad', ...
    'cg_body_x_m','cg_body_y_m','cg_body_z_m', ...
    'nozzle_angle_rad','thrust_normalized'});

writetable(T, csv_fullpath);
fprintf('CSV written to %s\n', csv_fullpath);
end
