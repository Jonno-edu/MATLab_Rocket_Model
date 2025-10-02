function generate_blender_csv_6dof(simOut)

% Output setup
csv_filename = 'trajectory_blender_6dof_60fps.csv';
output_folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'output');
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end
csv_fullpath = fullfile(output_folder, csv_filename);

% Access logsout
logsout = simOut.logsout;
plant_data = logsout.getElement('PlantData').Values;

% Position (NED)
Xe_ts = plant_data.Xe;
t_xe = Xe_ts.Time(:);
Xe_data = Xe_ts.Data;
if size(Xe_data,2) ~= 3
    error('PlantData.Xe Data is not N-by-3');
end
Xn_raw = Xe_data(:,1);
Xe_raw = Xe_data(:,2);
Xd_raw = Xe_data(:,3);

% Quaternion
q_be_ts = plant_data.q_be;
t_q_be = q_be_ts.Time(:);
q_be_data = q_be_ts.Data;
if size(q_be_data,2) ~= 4
    error('PlantData.q_be Data is not N-by-4');
end

% CG from tail
has_cg = false;
try
    cg_ts = logsout.getElement('CG_from_tail').Values;
    t_cg = cg_ts.Time(:);
    cg_raw = cg_ts.Data(:);
    has_cg = true;
catch
    cg_raw = [];
    t_cg = [];
end

% Nozzle angles
has_noz_y = false;
has_noz_z = false;
try
    nozY_ts = logsout.getElement('Y_nozzle_angle').Values;
    t_nozY = nozY_ts.Time(:);
    nozY_raw = nozY_ts.Data(:);
    has_noz_y = true;
catch
    nozY_raw = [];
    t_nozY = [];
end
try
    nozZ_ts = logsout.getElement('Z_nozzle_angle').Values;
    t_nozZ = nozZ_ts.Time(:);
    nozZ_raw = nozZ_ts.Data(:);
    has_noz_z = true;
catch
    nozZ_raw = [];
    t_nozZ = [];
end

% Create 60 FPS timebase
fps = 60;
t_start = max([t_xe(1), t_q_be(1)]);
t_end = min([t_xe(end), t_q_be(end)]);
if t_end <= t_start
    error('No overlapping time range between Xe and q_be');
end
t_uniform = (t_start : 1/fps : t_end).';

% Interpolate position
Xn = interp1(t_xe, Xn_raw, t_uniform, 'linear');
Xe = interp1(t_xe, Xe_raw, t_uniform, 'linear');
Xd = interp1(t_xe, Xd_raw, t_uniform, 'linear');

% Interpolate quaternion with SLERP
qobj_native = quaternion(q_be_data);
qobj_interp = interp1(t_q_be, qobj_native, t_uniform);
q_wxyz = compact(qobj_interp);
qw = q_wxyz(:,1);
qx = q_wxyz(:,2);
qy = q_wxyz(:,3);
qz = q_wxyz(:,4);

% Interpolate CG and nozzle angles
if has_cg
    cg_from_tail_m = interp1(t_cg, cg_raw, t_uniform, 'linear', 'extrap');
else
    cg_from_tail_m = zeros(size(t_uniform));
end
if has_noz_y
    y_nozzle_angle_rad = interp1(t_nozY, nozY_raw, t_uniform, 'linear', 'extrap');
else
    y_nozzle_angle_rad = zeros(size(t_uniform));
end
if has_noz_z
    z_nozzle_angle_rad = interp1(t_nozZ, nozZ_raw, t_uniform, 'linear', 'extrap');
else
    z_nozzle_angle_rad = zeros(size(t_uniform));
end

% Write CSV
T = table(t_uniform, Xn, Xe, Xd, qw, qx, qy, qz, ...
          cg_from_tail_m, y_nozzle_angle_rad, z_nozzle_angle_rad, ...
    'VariableNames', {'time_s','Xn_m','Xe_m','Xd_m','qw','qx','qy','qz', ...
                      'cg_from_tail_m','y_nozzle_angle_rad','z_nozzle_angle_rad'});
writetable(T, csv_fullpath);
fprintf('CSV written to %s\n', csv_fullpath);

end
