function success = generate_blender_csv(simOut, sim_params, filename_suffix)
% generate_blender_csv - Extracts key trajectory data and exports it to a CSV file for Blender.
%
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')
%           containing the logsout field.
%   sim_params: The simulation parameters struct, expected to contain
%               sim_params.OutputDataPath for the CSV output directory.
%   filename_suffix: (Optional) String to append to the default filename 
%                    (e.g., '_custom_run'). Default is '_60fps'.
%
% Outputs:
%   success: Logical true if CSV was generated successfully, false otherwise.

success = false; % Assume failure initially

if nargin < 3
    filename_suffix = '_60fps'; % Default suffix for Blender animation
end

disp('Attempting to generate Blender CSV...');

% --- Basic Input Validation ---
if ~exist('simOut', 'var') || isempty(simOut)
    disp('generate_blender_csv: simOut variable not provided or is empty.');
    return;
end
if ~exist('sim_params', 'var') || isempty(sim_params) || ~isfield(sim_params, 'OutputDataPath')
    disp('generate_blender_csv: sim_params not provided or OutputDataPath is missing.');
    return;
end
if ~isprop(simOut, 'logsout') || isempty(simOut.logsout)
    disp('generate_blender_csv: simOut does not contain logsout or logsout is empty. No CSV exported.');
    return;
end

logsout = simOut.logsout;

try
    % --- Access PlantData for key signals ---
    % This assumes 'PlantData' is a logged bus containing necessary signals.
    if any(strcmp(logsout.getElementNames(), 'PlantData'))
        plant_signal_object = logsout.getElement('PlantData');
        plant_data = plant_signal_object.Values; % This is the struct
    else
        disp('generate_blender_csv: "PlantData" bus not found in logsout. Cannot generate CSV.');
        return;
    end

    % --- Extract Time Vector ---
    % Choose a primary time vector. Here, we use time from Xe (position data).
    % Ensure this time vector covers the full simulation duration for Blender.
    if isfield(plant_data, 'Xe') && ~isempty(plant_data.Xe.Time)
        time_vec_raw = plant_data.Xe.Time;
    else
        disp('generate_blender_csv: Position data (PlantData.Xe.Time) not found or empty for time vector.');
        return;
    end
    
    % --- Define Desired Time Vector for Blender (e.g., 60 FPS) ---
    % You might want to resample to a fixed frame rate for animation.
    if ~isempty(sim_params.Sim) && isfield(sim_params.Sim, 'Time') && sim_params.Sim.Time > 0
        blender_frame_rate = 60; % Desired FPS for Blender
        t_blender = (0:(1/blender_frame_rate):sim_params.Sim.Time)'; % Ensure column vector
        if t_blender(end) < sim_params.Sim.Time && length(t_blender) > 1 % Add last sim time point if not too close
             t_blender = [t_blender; sim_params.Sim.Time];
             t_blender = unique(t_blender); % Ensure unique sorted times
        elseif isempty(t_blender) || length(t_blender) == 1
             t_blender = time_vec_raw; % Fallback to raw time if simTime is too short or 0
             disp('Using raw time vector for Blender CSV as simulation time is very short.');
        end

    else
        t_blender = time_vec_raw; % Fallback if sim_params.Sim.Time is not available
        disp('Simulation time not found in sim_params. Using raw time vector for Blender CSV.');
    end
    t_blender = t_blender(:); % Ensure column

    % --- Extract and Interpolate Pitch Angle ---
    % Assuming pitch is in PlantData.body_angles.theta (radians)
    if isfield(plant_data, 'body_angles') && isstruct(plant_data.body_angles) && isfield(plant_data.body_angles, 'theta')
        pitch_ts = plant_data.body_angles.theta;
        pitch_for_blender_rad = interp1(pitch_ts.Time, pitch_ts.Data, t_blender, 'linear', 'extrap');
        % Do not convert pitch to degrees; use radians for Blender animation
    else
        disp('generate_blender_csv: Pitch angle (PlantData.body_angles.theta) not found.');
        return;
    end

    % --- Extract and Interpolate Altitude ---
    % Assuming altitude is derived from Xe's Z-component (Z positive down in NED)
    % For Blender, typically positive Z is up.
    if isfield(plant_data, 'Xe')
        Xe_ts = plant_data.Xe;
        % Multiply by -1 if original Z is positive downwards and Blender needs positive up
        altitude_raw = -Xe_ts.Data(:,3); 
        altitude_for_blender = interp1(Xe_ts.Time, altitude_raw, t_blender, 'linear', 'extrap');
    else
        disp('generate_blender_csv: Position data (PlantData.Xe) for altitude not found.');
        return;
    end
    
    % --- You can add other signals here for Blender if needed ---
    % Example: Roll (phi) and Yaw (psi)
    % if isfield(plant_data.body_angles, 'phi')
    %     roll_ts = plant_data.body_angles.phi;
    %     roll_for_blender_deg = rad2deg(interp1(roll_ts.Time, roll_ts.Data, t_blender, 'linear', 'extrap'));
    % else
    %     roll_for_blender_deg = zeros(size(t_blender)); % Default to 0 if not found
    % end
    % if isfield(plant_data.body_angles, 'psi')
    %     yaw_ts = plant_data.body_angles.psi;
    %     yaw_for_blender_deg = rad2deg(interp1(yaw_ts.Time, yaw_ts.Data, t_blender, 'linear', 'extrap'));
    % else
    %     yaw_for_blender_deg = zeros(size(t_blender)); % Default to 0 if not found
    % end

    % --- Prepare data for CSV ---
    % Ensure all data are column vectors
    t_blender = t_blender(:);
    % Extract position data
    pos_x = Xe_ts.Data(:,1);
    pos_y = Xe_ts.Data(:,2);
    pos_z = altitude_for_blender(:);
    % Use pitch in radians
    pitch_rad = pitch_for_blender_rad(:);
    % Placeholder values for CG offsets, nozzle angle, and thrust normalization
    n = numel(t_blender);
    cg_body_x_m = zeros(n,1);
    cg_body_y_m = zeros(n,1);
    cg_body_z_m = zeros(n,1);
    nozzle_angle_rad = zeros(n,1);
    thrust_normalized = ones(n,1);

    % Define header names for Blender to easily identify columns
    header = {'time_s','pos_x_m','pos_y_m','pos_z_m','pitch_rad',...
              'cg_body_x_m','cg_body_y_m','cg_body_z_m',...
              'nozzle_angle_rad','thrust_normalized'};
    csvData = [t_blender, pos_x, pos_y, pos_z, pitch_rad,...
               cg_body_x_m, cg_body_y_m, cg_body_z_m,...
               nozzle_angle_rad, thrust_normalized];

    % --- Write to CSV file ---
    csvFilename = ['trajectory', filename_suffix, '.csv'];
    csvFilepath = fullfile(sim_params.OutputDataPath, csvFilename);
    
    T = array2table(csvData, 'VariableNames', header);
    writetable(T, csvFilepath);
    
    fprintf('Blender CSV successfully written to: %s\n', csvFilepath);
    success = true;

catch ME_csv
    disp('generate_blender_csv: Error during CSV generation.');
    disp(['Error Message: ', ME_csv.message]);
    fprintf('Stack Trace:\n');
    for k=1:length(ME_csv.stack)
        disp(ME_csv.stack(k));
    end
    success = false;
end

end
