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

t = plant_data.body_angles.theta.Time(:);
pos_x = plant_data.Xe.Data(:, 1);
pos_y = plant_data.Xe.Data(:, 2);
pos_z = -1 * plant_data.Xe.Data(:, 3);
pitch_rad = plant_data.body_angles.theta.Data(:);
cg_body_x = plant_data.CG.Data(:);
cg_body_y = zeros(size(t));
cg_body_z = zeros(size(t));
nozzle_angle_rad = logsout.getElement("Y_nozzle_angle").Values.Data;
thrust = logsout.getElement('engineThrust').Values.Data;
max_thrust = evalin('base', 'Actuators.Engine.MaxThrust');
if max_thrust > 0
    thrust_normalized = thrust / max_thrust;
else
    thrust_normalized = zeros(size(t));
end
% Resample all signals to 60 Hz (60 fps)
fps = 60;
t_uniform = (t(1):1/fps:t(end))';

% Interpolate all signals to uniform timebase
pos_x = interp1(t, pos_x, t_uniform, 'linear');
pos_y = interp1(t, pos_y, t_uniform, 'linear');
pos_z = interp1(t, pos_z, t_uniform, 'linear');
pitch_rad = interp1(t, pitch_rad, t_uniform, 'linear');
cg_body_x = interp1(t, cg_body_x, t_uniform, 'linear');
cg_body_y = interp1(t, cg_body_y, t_uniform, 'linear');
cg_body_z = interp1(t, cg_body_z, t_uniform, 'linear');
nozzle_angle_rad = interp1(t, nozzle_angle_rad, t_uniform, 'linear');
thrust_normalized = interp1(t, thrust_normalized, t_uniform, 'linear');

t = t_uniform;
N = length(t);
T = table(t, pos_x(1:N), pos_y(1:N), pos_z(1:N), pitch_rad(1:N), ...
    cg_body_x(1:N), cg_body_y(1:N), cg_body_z(1:N), ...
    nozzle_angle_rad(1:N), thrust_normalized(1:N), ...
    'VariableNames', {'time_s','pos_x_m','pos_y_m','pos_z_m','pitch_rad', ...
    'cg_body_x_m','cg_body_y_m','cg_body_z_m', ...
    'nozzle_angle_rad','thrust_normalized'});

writetable(T, csv_fullpath);
fprintf('CSV written to %s\n', csv_fullpath);
end
