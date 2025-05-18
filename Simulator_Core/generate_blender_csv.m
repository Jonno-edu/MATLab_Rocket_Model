function success = generate_blender_csv(simOut, sim_params, filename_suffix)
% generate_blender_csv - Extracts key trajectory data and exports it to a CSV file
%                        formatted for the Blender Python animation script.
%
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')
%           containing the logsout field.
%   sim_params: The simulation parameters struct, expected to contain:
%               - sim_params.OutputDataPath (for the CSV output directory)
%               - sim_params.Sim.Time (for total simulation time)
%               - (Optional) sim_params.Propulsion.MaxThrust (for thrust normalization)
%   filename_suffix: (Optional) String to append to the default filename.
%                    Default is '_60fps'.
%
% Outputs:
%   success: Logical true if CSV was generated successfully, false otherwise.

success = false; % Assume failure initially

% --- Handle Optional Suffix Argument ---
if nargin < 3 || isempty(filename_suffix)
    filename_suffix = '_60fps'; % Default suffix
end

disp('--- Starting Blender CSV Generation Function ---');

% --- Configuration from sim_params ---
% Use OutputDataPath from sim_params; fallback to script's directory if not found
if isfield(sim_params, 'OutputDataPath') && ~isempty(sim_params.OutputDataPath) && exist(sim_params.OutputDataPath, 'dir')
    output_directory = sim_params.OutputDataPath;
else
    script_full_path = mfilename('fullpath');
    [script_directory, ~, ~] = fileparts(script_full_path);
    output_directory = fullfile(script_directory, 'output_data'); % Default to an 'output_data' subfolder
    fprintf('Warning: sim_params.OutputDataPath not found or invalid. Using default: %s\n', output_directory);
    if ~exist(output_directory, 'dir')
        try
            mkdir(output_directory);
            fprintf('Created default output directory: %s\n', output_directory);
        catch ME_mkdir
            error('MATLAB:FunctionError', 'Could not create default output directory "%s". Error: %s.', output_directory, ME_mkdir.message);
        end
    end
end

base_csv_filename = 'trajectory_for_blender_60fps'; % Base name for the CSV
blender_frame_rate = 60; % Desired FPS for Blender animation

% --- Initial Checks ---
if ~exist('simOut', 'var') || isempty(simOut)
    error('MATLAB:FunctionError', 'simOut variable not provided or is empty.');
end
if ~isprop(simOut, 'logsout') || isempty(simOut.logsout)
    error('MATLAB:FunctionError', 'simOut.logsout not found or is empty. Check signal logging in Simulink.');
end

logsout = simOut.logsout;
csv_filepath = fullfile(output_directory, [base_csv_filename, filename_suffix, '.csv']);

try
    % --- Access PlantData for key signals ---
    if any(strcmp(logsout.getElementNames(), 'PlantData'))
        plant_signal_object = logsout.getElement('PlantData');
        pd = plant_signal_object.Values; % PlantData struct
    else
        error('MATLAB:FunctionError', '"PlantData" bus not found in logsout. Cannot generate CSV.');
    end

    % --- Determine Simulation End Time from sim_params and Define Time Vector for Blender ---
    if isfield(sim_params, 'Sim') && isfield(sim_params.Sim, 'Time') && sim_params.Sim.Time > 0
        t_sim_end = sim_params.Sim.Time;
    else
        % Fallback to inferring from data if sim_params.Sim.Time is missing/invalid
        if isfield(pd, 'Xe') && isa(pd.Xe, 'timeseries') && ~isempty(pd.Xe.Time)
            raw_time_vector_for_duration = pd.Xe.Time;
            if ~isempty(raw_time_vector_for_duration)
                t_sim_end = raw_time_vector_for_duration(end);
                fprintf('Warning: Using simulation duration inferred from PlantData.Xe.Time (%.2f s) as sim_params.Sim.Time was invalid.\n', t_sim_end);
            else
                 error('MATLAB:FunctionError', 'sim_params.Sim.Time is invalid and PlantData.Xe.Time is also unavailable to determine duration.');
            end
        else
            error('MATLAB:FunctionError', 'sim_params.Sim.Time is invalid and PlantData.Xe is unavailable to determine duration.');
        end
    end
    
    t_blender = (0:(1/blender_frame_rate):t_sim_end)';
    if t_blender(end) < t_sim_end && (t_sim_end - t_blender(end)) > (0.5/blender_frame_rate)
         t_blender = [t_blender; t_sim_end];
    end
    t_blender = unique(t_blender);
    t_blender = t_blender(:);

    % --- Extract and Interpolate Required Signals ---

    % 1. Position Data (pos_x_m, pos_y_m, pos_z_m) - World CG
    if isfield(pd, 'Xe') && isa(pd.Xe, 'timeseries')
        Xe_ts = pd.Xe;
        if size(Xe_ts.Data,2) < 3
             error('MATLAB:FunctionError', 'PlantData.Xe.Data does not have at least 3 columns for X,Y,Z position.');
        end
        pos_x_m_interp = checked_interp1(Xe_ts.Time, double(Xe_ts.Data(:,1)), t_blender, 'Xe_X');
        pos_y_m_interp = checked_interp1(Xe_ts.Time, double(Xe_ts.Data(:,2)), t_blender, 'Xe_Y');
        pos_z_m_interp = checked_interp1(Xe_ts.Time, double(-Xe_ts.Data(:,3)), t_blender, 'Xe_Z (inverted)'); % Z positive UP
    else
        error('MATLAB:FunctionError', 'Position data (PlantData.Xe as timeseries) not found.');
    end

    % 2. Pitch Angle (pitch_rad)
    if isfield(pd, 'body_angles') && isstruct(pd.body_angles) && isfield(pd.body_angles, 'theta') && isa(pd.body_angles.theta, 'timeseries')
        pitch_ts = pd.body_angles.theta;
        pitch_rad_interp = checked_interp1(pitch_ts.Time, double(pitch_ts.Data), t_blender, 'Pitch (theta)');
    else
        error('MATLAB:FunctionError', 'Pitch angle (PlantData.body_angles.theta as timeseries) not found.');
    end

    % 3. CG in Body Frame (cg_body_x_m, cg_body_y_m, cg_body_z_m) - Offset from NOSE
    fprintf('\n--- Processing PlantData.CG for interpolation ---\n');
    if isfield(pd, 'CG') && isa(pd.CG, 'timeseries')
        cg_ts = pd.CG;
        fprintf('PlantData.CG is a timeseries.\n');
        fprintf('  Class of cg_ts.Time: %s, Size: %s\n', class(cg_ts.Time), mat2str(size(cg_ts.Time)));
        fprintf('  Class of cg_ts.Data: %s, Size: %s\n', class(cg_ts.Data), mat2str(size(cg_ts.Data)));
        
        cg_data_double = double(squeeze(cg_ts.Data)); 
        fprintf('  Squeezed and Converted cg_ts.Data to double. New Class: %s, Size: %s\n', class(cg_data_double), mat2str(size(cg_data_double)));

        if isnumeric(cg_data_double) 
            if size(cg_data_double, 2) == 3 
                fprintf('  cg_data_double is numeric with 3 columns. Proceeding with 3D CG.\n');
                cg_body_x_m_interp = checked_interp1(cg_ts.Time, cg_data_double(:,1), t_blender, 'CG_body_X');
                cg_body_y_m_interp = checked_interp1(cg_ts.Time, cg_data_double(:,2), t_blender, 'CG_body_Y');
                cg_body_z_m_interp = checked_interp1(cg_ts.Time, cg_data_double(:,3), t_blender, 'CG_body_Z');
            elseif isvector(cg_data_double) 
                fprintf('  cg_data_double is a vector. Assuming longitudinal CG (X).\n');
                cg_body_x_m_interp = checked_interp1(cg_ts.Time, cg_data_double(:), t_blender, 'CG_body_X (1D)');
                cg_body_y_m_interp = zeros(size(t_blender));
                cg_body_z_m_interp = zeros(size(t_blender));
                disp('Warning: CG data (PlantData.CG) processed as 1D. Assuming it is cg_body_x_m; y/z offsets set to 0.');
            else
                disp('Warning: PlantData.CG.Data (squeezed & double) has unexpected dimensions. Setting body CG offsets to 0.');
                cg_body_x_m_interp = zeros(size(t_blender));
                cg_body_y_m_interp = zeros(size(t_blender));
                cg_body_z_m_interp = zeros(size(t_blender));
            end
        else
             error('MATLAB:FunctionError', 'PlantData.CG.Data after squeeze/double conversion is not numeric.');
        end
    else
        disp('Warning: CG data (PlantData.CG as timeseries) not found or not a timeseries. Setting body CG offsets to 0.');
        cg_body_x_m_interp = zeros(size(t_blender));
        cg_body_y_m_interp = zeros(size(t_blender));
        cg_body_z_m_interp = zeros(size(t_blender));
    end
    fprintf('--- Finished processing PlantData.CG ---\n\n');

    % 4. Nozzle Angle (nozzle_angle_rad)
    nozzle_angle_rad_interp = zeros(size(t_blender)); 
    found_nozzle_source = 'None';
    if isfield(pd, 'Actuators') && isstruct(pd.Actuators) && isfield(pd.Actuators, 'Y_nozzle_angle') && isa(pd.Actuators.Y_nozzle_angle, 'timeseries')
        nozzle_ts = pd.Actuators.Y_nozzle_angle;
        nozzle_angle_rad_interp = checked_interp1(nozzle_ts.Time, double(squeeze(nozzle_ts.Data)), t_blender, 'Nozzle (PlantData.Actuators)');
        found_nozzle_source = 'PlantData.Actuators.Y_nozzle_angle';
    elseif any(strcmp(logsout.getElementNames(), 'Y_nozzle_angle'))
        nozzle_obj_top = logsout.getElement('Y_nozzle_angle');
        if isa(nozzle_obj_top.Values, 'timeseries')
            nozzle_ts_top = nozzle_obj_top.Values;
            nozzle_angle_rad_interp = checked_interp1(nozzle_ts_top.Time, double(squeeze(nozzle_ts_top.Data)), t_blender, 'Nozzle (Top-Level)');
            found_nozzle_source = 'Top-Level Y_nozzle_angle';
        else
            disp('Warning: Top-level Y_nozzle_angle found but is not a timeseries. Setting to 0.');
        end
    else
        disp('Warning: Nozzle angle (Y_nozzle_angle) not found. Setting to 0 for CSV.');
    end
    fprintf('Using nozzle angle from: %s for CSV.\n', found_nozzle_source);

    % 5. Normalized Thrust (thrust_normalized)
    thrust_normalized_interp = zeros(size(t_blender)); 
    if isfield(pd, 'Actuators') && isstruct(pd.Actuators) && isfield(pd.Actuators, 'engineThrust') && isa(pd.Actuators.engineThrust, 'timeseries')
        thrust_ts = pd.Actuators.engineThrust;
        engine_thrust_data_raw = squeeze(double(thrust_ts.Data));
        fprintf('  EngineThrust original data size: %s, Squeezed data size: %s\n', ...
                mat2str(size(thrust_ts.Data)), mat2str(size(engine_thrust_data_raw)));
        actual_thrust_N = checked_interp1(thrust_ts.Time, engine_thrust_data_raw, t_blender, 'EngineThrust (Squeezed)');
        
        max_thrust_for_norm = 0;
        if isfield(sim_params, 'Propulsion') && isfield(sim_params.Propulsion, 'MaxThrust') && sim_params.Propulsion.MaxThrust > 0
            max_thrust_for_norm = sim_params.Propulsion.MaxThrust;
            fprintf('Using MaxThrust from sim_params for normalization: %.2f N\n', max_thrust_for_norm);
        else
            max_thrust_for_norm = max(engine_thrust_data_raw);
             if isempty(max_thrust_for_norm) || max_thrust_for_norm <= 0 
                max_thrust_for_norm = 1.0; 
                disp('Warning: Max logged thrust for normalization is zero, negative or empty. Using 1.0 as denominator.');
             else
                fprintf('Using max logged thrust for normalization: %.2f N\n', max_thrust_for_norm);
             end
        end
        
        thrust_normalized_interp = actual_thrust_N / max_thrust_for_norm;
        thrust_normalized_interp(thrust_normalized_interp < 0) = 0; 
        thrust_normalized_interp(thrust_normalized_interp > 1) = 1; 
        disp('Normalized thrust calculated for CSV.');
    else
        disp('Warning: Engine thrust (PlantData.Actuators.engineThrust as timeseries) not found. Setting normalized thrust to 0 for CSV.');
    end

    % --- Prepare data for CSV ---
    pos_x_m_interp = pos_x_m_interp(:);
    pos_y_m_interp = pos_y_m_interp(:);
    pos_z_m_interp = pos_z_m_interp(:);
    pitch_rad_interp = pitch_rad_interp(:);
    cg_body_x_m_interp = cg_body_x_m_interp(:);
    cg_body_y_m_interp = cg_body_y_m_interp(:);
    cg_body_z_m_interp = cg_body_z_m_interp(:);
    nozzle_angle_rad_interp = nozzle_angle_rad_interp(:);
    thrust_normalized_interp = thrust_normalized_interp(:);

    header = {'time_s', 'pos_x_m', 'pos_y_m', 'pos_z_m', ...
              'pitch_rad', ...
              'cg_body_x_m', 'cg_body_y_m', 'cg_body_z_m', ...
              'nozzle_angle_rad', 'thrust_normalized'};
              
    csvDataMatrix = [t_blender, pos_x_m_interp, pos_y_m_interp, pos_z_m_interp, ...
                     pitch_rad_interp, ...
                     cg_body_x_m_interp, cg_body_y_m_interp, cg_body_z_m_interp, ...
                     nozzle_angle_rad_interp, thrust_normalized_interp];

    % --- Write to CSV file ---
    T = array2table(csvDataMatrix, 'VariableNames', header);
    writetable(T, csv_filepath);
    
    fprintf('Blender CSV successfully written to: %s\n', csv_filepath);
    disp('--- Blender CSV Generation Function Finished ---');
    success = true; % Set success flag

catch ME_csv
    fprintf(2, 'Error during Blender CSV generation function:\n'); 
    fprintf(2, 'Error Message: %s\n', ME_csv.message);
    if ~isempty(ME_csv.stack)
        fprintf(2, 'Error occurred in %s at line %d.\n', ME_csv.stack(1).name, ME_csv.stack(1).line);
    end
    success = false; % Ensure success is false on error
end

end % END OF MAIN FUNCTION generate_blender_csv

% --- Local Helper Function Definition (at the end of the file) ---
function Vq = checked_interp1(X_ts_Time, V_ts_Data_Column, Xq, signal_name_for_error)
    fprintf('Inspecting data for interp1 on "%s":\n', signal_name_for_error);
    fprintf('  Input Time (X_ts_Time) - Class: %s, Size: %s\n', class(X_ts_Time), mat2str(size(X_ts_Time)));
    fprintf('  Input Data (V_ts_Data_Column) - Class: %s, Size: %s\n', class(V_ts_Data_Column), mat2str(size(V_ts_Data_Column)));

    X_ts_Time_double = double(X_ts_Time); 
    V_ts_Data_Column_squeezed_double = double(squeeze(V_ts_Data_Column)); 

    fprintf('  Squeezed & Doubled Input Data (V_ts_Data_Column_squeezed_double) - Class: %s, Size: %s\n', ...
            class(V_ts_Data_Column_squeezed_double), mat2str(size(V_ts_Data_Column_squeezed_double)));

    if ~isvector(X_ts_Time_double) || isempty(X_ts_Time_double)
         error('MATLAB:FunctionError:checked_interp1', 'Time vector for "%s" is not a non-empty vector after double conversion. Original Class: %s, Size: %s', ...
               signal_name_for_error, class(X_ts_Time), mat2str(size(X_ts_Time)));
    end
    if ~isvector(V_ts_Data_Column_squeezed_double) || isempty(V_ts_Data_Column_squeezed_double)
         error('MATLAB:FunctionError:checked_interp1', 'Data column for "%s" is not a non-empty vector after squeeze & double conversion. Original Class: %s, Original Size: %s, Squeezed Size: %s', ...
               signal_name_for_error, class(V_ts_Data_Column), mat2str(size(V_ts_Data_Column)), mat2str(size(V_ts_Data_Column_squeezed_double)));
    end
    
    X_ts_Time_col = X_ts_Time_double(:);
    V_ts_Data_Column_col = V_ts_Data_Column_squeezed_double(:);

    fprintf('  Dimensions for interp1 on "%s" (after ensuring columns & double & squeeze): Time length = %d, Data length = %d\n', ...
            signal_name_for_error, length(X_ts_Time_col), length(V_ts_Data_Column_col));
            
    if length(X_ts_Time_col) ~= length(V_ts_Data_Column_col)
        error('MATLAB:FunctionError:checked_interp1', 'For signal "%s", Time vector length (%d) and Data vector length (%d) mismatch for interp1.', ...
              signal_name_for_error, length(X_ts_Time_col), length(V_ts_Data_Column_col));
    end
    
    if any(isnan(X_ts_Time_col)) || any(isinf(X_ts_Time_col))
        warning('MATLAB:ScriptWarning:checked_interp1', 'Time vector for "%s" contains NaNs or Infs.', signal_name_for_error);
    end
    if any(isnan(V_ts_Data_Column_col)) || any(isinf(V_ts_Data_Column_col))
        warning('MATLAB:ScriptWarning:checked_interp1', 'Data vector for "%s" contains NaNs or Infs.', signal_name_for_error);
    end

    Vq = interp1(X_ts_Time_col, V_ts_Data_Column_col, Xq, 'linear', 'extrap');
end
