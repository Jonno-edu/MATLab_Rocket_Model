function generate_blender_csv(simOut)
% Generate a CSV for Blender animation from simOut, including forces, CP, alpha, and thrust

csv_filename = 'trajectory_for_blender_60fps.csv';
output_folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'output');
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end
csv_fullpath = fullfile(output_folder, csv_filename);

logsout = simOut.logsout;

% --- PlantData signals (nested BUS) ---
try
    plant_data = logsout.getElement('PlantData').Values;
catch
    error('PlantData not found in simOut.logsout!');
end
t_main = plant_data.body_angles.theta.Time(:);

% Positions
try
    pos_x = plant_data.Xe.Data(:, 1);
    pos_y = plant_data.Xe.Data(:, 2);
    pos_z = -1 * plant_data.Xe.Data(:, 3);
    t_pos = t_main;
catch
    warning('Xe position not found in PlantData - using zeros');
    pos_x = zeros(size(t_main));
    pos_y = zeros(size(t_main));
    pos_z = zeros(size(t_main));
    t_pos = t_main;
end

% Pitch and Alpha angles
try
    pitch_rad = plant_data.body_angles.theta.Data(:);
    t_pitch = t_main;
catch
    warning('Pitch not found - using zeros');
    pitch_rad = zeros(size(t_main));
    t_pitch = t_main;
end

try
    alpha_rad = plant_data.alpha.Data(:);
    t_alpha = plant_data.alpha.Time(:);
catch
    warning('Angle of attack (alpha) not found - using zeros');
    alpha_rad = zeros(size(t_main));
    t_alpha = t_main;
end

% Center of gravity
try
    cg_body_x = plant_data.CG.Data(:);
    t_cg = t_main;
catch
    warning('CG data not found - using zeros');
    cg_body_x = zeros(size(t_main));
    t_cg = t_main;
end

% --- Nozzle angle (top level) ---
try
    nozzle_data = logsout.getElement('Y_nozzle_angle').Values;
    nozzle_angle_rad = nozzle_data.Data(:);
    t_nozzle = nozzle_data.Time(:);
catch
    warning('Nozzle angle (Y_nozzle_angle) not found - using zeros');
    nozzle_angle_rad = zeros(size(t_main));
    t_nozzle = t_main;
end
min_len_nozzle = min(length(t_nozzle), length(nozzle_angle_rad));
t_nozzle = t_nozzle(1:min_len_nozzle);
nozzle_angle_rad = nozzle_angle_rad(1:min_len_nozzle);

% --- Thrust normalized and absolute (new) ---
try
    thrust_data = logsout.getElement('engineThrust').Values;
    thrust = thrust_data.Data(:);
    t_thrust = thrust_data.Time(:);

    max_thrust = evalin('base', 'Actuators.Engine.MaxThrust');
    if max_thrust > 0
        thrust_normalized = thrust / max_thrust;
    else
        thrust_normalized = zeros(size(thrust));
    end
catch
    warning('engineThrust not found - using zeros');
    thrust = zeros(size(t_main));
    thrust_normalized = zeros(size(t_main));
    t_thrust = t_main;
end

% --- CP (from nose, top level) ---
try
    cp_from_nose = logsout.getElement('CP (from nose)').Values;
    cp_from_nose_data = cp_from_nose.Data(:);
    t_cp = cp_from_nose.Time(:);
catch
    warning('CP (from nose) not found - using zeros');
    cp_from_nose_data = zeros(size(t_main));
    t_cp = t_main;
end

% --- ENV BUS: Fg (gravity body axis) ---
try
    ENV_struct = logsout.getElement('ENV').Values;
    Fg_ts = ENV_struct.Fg;
    Fg_x = Fg_ts.Data(:,1);
    Fg_z = Fg_ts.Data(:,3);
    t_Fg = Fg_ts.Time(:);
catch
    warning('Fg not found in ENV - using zeros');
    Fg_x = zeros(size(t_main));
    Fg_z = zeros(size(t_main));
    t_Fg = t_main;
end

% --- F_aero (top level) ---
try
    F_aero = logsout.getElement('F_aero').Values;
    F_aero_x = F_aero.Data(:,1);
    F_aero_z = F_aero.Data(:,3);
    t_Faero = F_aero.Time(:);
catch
    warning('F_aero not found - using zeros');
    F_aero_x = zeros(size(t_main));
    F_aero_z = zeros(size(t_main));
    t_Faero = t_main;
end

% --- Normal Force Aero (top level) ---
try
    nf_aero = logsout.getElement('Normal Force Aero').Values;
    NormalForceAero = nf_aero.Data(:);
    t_nfaero = nf_aero.Time(:);
catch
    warning('Normal Force Aero not found - using zeros');
    NormalForceAero = zeros(size(t_main));
    t_nfaero = t_main;
end

% --- Axial Force Aero (top level) ---
try
    ax_aero = logsout.getElement('Axial Force Aero').Values;
    AxialForceAero = ax_aero.Data(:);
    t_axaero = ax_aero.Time(:);
catch
    warning('Axial Force Aero not found - using zeros');
    AxialForceAero = zeros(size(t_main));
    t_axaero = t_main;
end

% --- Common resampling timebase ---
t_start = max([t_main(1), t_pos(1), t_pitch(1), t_cg(1), t_nozzle(1), t_thrust(1)]);
t_end = min([t_main(end), t_pos(end), t_pitch(end), t_cg(end), t_nozzle(end), t_thrust(end)]);
fps = 60;
t_uniform = (t_start:1/fps:t_end)';

% --- Interpolated columns ---
pos_x_interp = interp1(t_pos, pos_x, t_uniform, 'linear');
pos_y_interp = interp1(t_pos, pos_y, t_uniform, 'linear');
pos_z_interp = interp1(t_pos, pos_z, t_uniform, 'linear');
pitch_rad_interp = interp1(t_pitch, pitch_rad, t_uniform, 'linear');
alpha_rad_interp = interp1(t_alpha, alpha_rad, t_uniform, 'linear');
cg_body_x_interp = interp1(t_cg, cg_body_x, t_uniform, 'linear');
cg_body_y_interp = zeros(size(t_uniform));
cg_body_z_interp = zeros(size(t_uniform));
nozzle_angle_rad_interp = interp1(t_nozzle, nozzle_angle_rad, t_uniform, 'linear');
thrust_normalized_interp = interp1(t_thrust, thrust_normalized, t_uniform, 'linear');
thrust_interp = interp1(t_thrust, thrust, t_uniform, 'linear'); % <--- NEW: Thrust (N)
cp_from_nose_interp = interp1(t_cp, cp_from_nose_data, t_uniform, 'linear');
Fg_x_interp = interp1(t_Fg, Fg_x, t_uniform, 'linear');
Fg_z_interp = interp1(t_Fg, Fg_z, t_uniform, 'linear');
F_aero_x_interp = interp1(t_Faero, F_aero_x, t_uniform, 'linear');
F_aero_z_interp = interp1(t_Faero, F_aero_z, t_uniform, 'linear');
NormalForceAero_interp = interp1(t_nfaero, NormalForceAero, t_uniform, 'linear');
AxialForceAero_interp = interp1(t_axaero, AxialForceAero, t_uniform, 'linear');
Drag_interp = -F_aero_x_interp;
Lift_interp = F_aero_z_interp;

% --- Table for CSV ---
T = table(...
    t_uniform, pos_x_interp, pos_y_interp, pos_z_interp, pitch_rad_interp, ...
    alpha_rad_interp, ...
    cg_body_x_interp, cg_body_y_interp, cg_body_z_interp, ...
    nozzle_angle_rad_interp, thrust_normalized_interp, thrust_interp, ... % <--- Thrust added here
    cp_from_nose_interp, ...
    Fg_x_interp, Fg_z_interp, ...
    F_aero_x_interp, F_aero_z_interp, ...
    NormalForceAero_interp, AxialForceAero_interp, ...
    Drag_interp, Lift_interp, ...
    'VariableNames', { ...
        'time_s','pos_x_m','pos_y_m','pos_z_m','pitch_rad', ...
        'alpha_rad', ...
        'cg_body_x_m','cg_body_y_m','cg_body_z_m', ...
        'nozzle_angle_rad','thrust_normalized','thrust_N', ...
        'cp_from_nose_m', ...
        'Fg_x_N','Fg_z_N', ...
        'F_aero_x_N','F_aero_z_N', ...
        'NormalForceAero_N','AxialForceAero_N', ...
        'Drag_N','Lift_N' ...
    });

writetable(T, csv_fullpath);
fprintf('CSV written to %s\n', csv_fullpath);

end
