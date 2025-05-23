% --- MATLAB Script for System Identification from CSV ---
% Imports data, resamples to uniform grid, defines a step input,
% and creates an iddata object for use in the System Identification app.
% Assumes time in CSV is in milliseconds (ms) and output is in millimeters (mm).
% All time units are converted to SECONDS for processing and iddata creation.
% Step input is defined to occur at 20 ms with an amplitude of 4.2 mm.

%% 1. Define File Path and Import Options
% Define the path to your CSV file.
% Ensure this file is in your MATLAB Current Folder or provide the full path.
csvFilePath = 'plot-data.csv';

% Set up import options for a CSV with a header row and two data columns.
opts = delimitedTextImportOptions("NumVariables", 2);
opts.DataLines = [2, Inf]; % Data starts on the second line.
opts.Delimiter = ",";
% Define variable names based on assumed content: time in ms, output in mm.
opts.VariableNames = ["time_from_csv_ms", "output_from_csv_mm"];
opts.VariableTypes = ["double", "double"]; % Both columns are numeric.
opts.ExtraColumnsRule = "ignore"; % Ignore any extra columns.
opts.EmptyLineRule = "read"; % Read empty lines.

%% 2. Import Data from CSV
try
    dataTable = readtable(csvFilePath, opts);
    fprintf('Data successfully imported from: %s\n', csvFilePath);
catch ME
    fprintf('Error importing data from %s:\n', csvFilePath);
    fprintf('Error message: %s\n', ME.message);
    fprintf('Please ensure the file exists in the MATLAB path or current folder, is not open elsewhere, and is correctly formatted.\n');
    fprintf('Script will terminate.\n');
    return; % Stop script execution if data import fails.
end

% Extract data into separate column vectors.
original_time_ms = dataTable.time_from_csv_ms;     % Time data from CSV (assumed in milliseconds).
original_output_mm = dataTable.output_from_csv_mm; % Output data from CSV (assumed in millimeters).

% --- Crucial Step: Convert original time from milliseconds to SECONDS ---
original_time_seconds = original_time_ms / 1000;

% Display a sample of the imported and converted data to verify.
disp('--- Sample of Imported Data ---');
disp('First 5 rows of original time data (from CSV, in ms):');
disp(head(original_time_ms, 5));
disp('First 5 rows of time data (converted to seconds):');
disp(head(original_time_seconds, 5));
disp('First 5 rows of output data (from CSV, in mm):');
disp(head(original_output_mm, 5));
disp('-------------------------------');

%% 3. Resample Data to a Uniform Time Grid (if original is non-uniform)
% Ensure data vectors are column vectors for consistency.
original_time_seconds = original_time_seconds(:);
original_output_mm = original_output_mm(:);

% Check if data is effectively empty after import.
if isempty(original_time_seconds) || isempty(original_output_mm)
    fprintf('Error: Imported time or output data is empty. Cannot proceed.\n');
    return;
end

% 3.1. Define a desired uniform sampling interval (Ts_uniform) in SECONDS.
if length(original_time_seconds) > 1
    % Calculate the average time difference between original samples.
    avg_original_dt_seconds = mean(diff(original_time_seconds));
    % Heuristic: Set uniform sampling interval to half the average original interval.
    % This aims to preserve dynamics without oversampling excessively. Adjust as needed.
    Ts_uniform_seconds = avg_original_dt_seconds / 2;
    
    % Sanity check for Ts_uniform_seconds.
    if Ts_uniform_seconds <= 0 || isnan(Ts_uniform_seconds)
        fprintf('Warning: Calculated Ts_uniform_seconds (%.6f) is non-positive or NaN. \n', Ts_uniform_seconds);
        % Fallback: Use a fraction of the total duration or an absolute small value.
        Ts_uniform_seconds = (original_time_seconds(end) - original_time_seconds(1)) / (length(original_time_seconds) * 10 + 1);
        if Ts_uniform_seconds <= 0 || isnan(Ts_uniform_seconds), Ts_uniform_seconds = 0.001; end % Absolute fallback (1 ms).
        fprintf('Using fallback Ts_uniform_seconds: %.6f seconds\n', Ts_uniform_seconds);
    end
else
    % If only one data point, set a default sampling interval (e.g., 1 ms).
    % System identification with one point is generally not meaningful.
    Ts_uniform_seconds = 0.001;
    fprintf('Warning: Only one data point found. Using default Ts_uniform_seconds: %.6f seconds\n', Ts_uniform_seconds);
end
fprintf('Target uniform sampling interval (Ts_uniform): %.6f seconds\n', Ts_uniform_seconds);

% 3.2. Create a new uniform time vector spanning the original time range, in SECONDS.
% Ensure start and end times are valid.
if isempty(original_time_seconds)
    fprintf('Error: Original time vector (seconds) is empty before creating uniform time vector.\n');
    return;
end
uniform_time_seconds = (original_time_seconds(1) : Ts_uniform_seconds : original_time_seconds(end))';

% Handle cases where the uniform vector might be empty (e.g., start and end are identical, Ts is too large).
if isempty(uniform_time_seconds) && ~isempty(original_time_seconds)
    uniform_time_seconds = original_time_seconds(1); % Use at least the start time.
    fprintf('Warning: Uniform time vector was empty; set to start time only.\n');
end


% 3.3. Resample the output data to the new uniform time vector using linear interpolation.
% 'extrap' is used to handle query points slightly outside the original data range if any.
resampled_output_mm = interp1(original_time_seconds, original_output_mm, uniform_time_seconds, 'linear', 'extrap');

% 3.4. Clean up potential NaNs from interpolation (less likely with 'extrap' but good practice).
valid_indices = ~isnan(resampled_output_mm);
uniform_time_seconds_cleaned = uniform_time_seconds(valid_indices);
resampled_output_mm_cleaned = resampled_output_mm(valid_indices);

% Check if data became empty after cleaning.
if isempty(uniform_time_seconds_cleaned)
    fprintf('Error: All resampled data points resulted in NaN or an empty set after cleaning. \n');
    fprintf('Check original data quality and resampling parameters (especially Ts_uniform).\n');
    return;
end

fprintf('Data resampled to a uniform grid with %d points.\n', length(uniform_time_seconds_cleaned));

%% 4. Define the Step Input and Create the `iddata` Object

% 4.1. Define parameters for the step input.
step_time_seconds = 0.020;  % Step input occurs at 20 ms (0.020 seconds).
step_amplitude_mm = 4.2;    % Amplitude of the step input in mm.

% 4.2. Create the step input vector.
% The input signal is 0 before step_time_seconds and step_amplitude_mm at and after.
step_input_mm = zeros(size(uniform_time_seconds_cleaned));
step_input_mm(uniform_time_seconds_cleaned >= step_time_seconds) = step_amplitude_mm;

% 4.3. Determine the final sampling interval (Ts) for the `iddata` object (in SECONDS).
% This should be very close to Ts_uniform_seconds unless many NaNs were removed.
if length(uniform_time_seconds_cleaned) > 1
    Ts_for_iddata = mean(diff(uniform_time_seconds_cleaned));
    % If mean diff results in NaN (e.g. if only one point left), revert.
    if isnan(Ts_for_iddata) || Ts_for_iddata <= 0
        Ts_for_iddata = Ts_uniform_seconds;
    end
else % Only one data point remaining after cleaning.
    Ts_for_iddata = Ts_uniform_seconds; % Use the target Ts.
    fprintf('Warning: Only one data point remains after cleaning. System ID might be problematic.\n');
end

% 4.4. Create the `iddata` object [2].
% `iddata(OutputData, InputData, SampleTime, 'Tstart', StartTime)`
data_step_for_app = iddata(resampled_output_mm_cleaned, ... % Output signal (mm).
                           step_input_mm, ...                % Input signal (mm).
                           Ts_for_iddata, ...                % Sampling interval (seconds).
                           'Tstart', uniform_time_seconds_cleaned(1)); % Start time (seconds).

% 4.5. Assign metadata (names and units) to the `iddata` object for clarity in the app [2].
data_step_for_app.Name = 'ActuatorStepResponse_20ms_4.2mm'; % Descriptive name for the dataset.
data_step_for_app.TimeUnit = 'seconds';                   % Unit for time.

data_step_for_app.InputName = {'StepInput'};              % Name for the input channel.
data_step_for_app.InputUnit = {'mm'};                     % Unit for the input signal.

data_step_for_app.OutputName = {'ActuatorOutput'};        % Name for the output channel.
data_step_for_app.OutputUnit = {'mm'};                    % Unit for the output signal.

%% 5. Display Information and Next Steps
disp(' '); % Add a blank line for readability.
disp('--- `iddata` Object Created ---');
disp(['  Name: ', data_step_for_app.Name]);
disp(['  Sampling Interval (Ts): ', num2str(data_step_for_app.Ts), ' ', data_step_for_app.TimeUnit]);
disp(['  Number of Samples: ', num2str(length(data_step_for_app.OutputData))]);
disp(['  Start Time (Tstart): ', num2str(data_step_for_app.Tstart), ' ', data_step_for_app.TimeUnit]);
disp(['  Input Channel: ', data_step_for_app.InputName{1}, ' (Unit: ', data_step_for_app.InputUnit{1}, ')']);
disp(['  Output Channel: ', data_step_for_app.OutputName{1}, ' (Unit: ', data_step_for_app.OutputUnit{1}, ')']);
disp(['  Step input defined to occur at t = ', num2str(step_time_seconds), ' seconds with an amplitude of ', num2str(step_amplitude_mm), ' mm.']);
disp('-------------------------------');
disp(' ');
disp('Next Steps:');
disp('1. Open the System Identification app in MATLAB: `systemIdentification`');
disp('2. In the app: Click on "Import data" > "Data object".');
disp('3. Enter the object name: `data_step_for_app` and click "Import".');
disp('4. Proceed to "Estimate" > "Transfer Function Models..." to identify your system.');
disp('   The time axes in plots should now be in seconds, and signal axes in mm.');

% --- End of Script ---
