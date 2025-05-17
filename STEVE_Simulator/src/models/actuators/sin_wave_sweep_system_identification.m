% --- Simplified Script to Run Simulink and Export Specific Data ---
clear; clc; close all;


% --- Load and Simulate the Model ---
modelName = 'second_order_sine_sweep'; % Just the name, no .slx


disp(['Loading Simulink model from: ', modelFileWithPath, '...']);
load_system(modelName);

simulation_time_duration = 120;

% --- Extract Desired Data ---
dataError = false;
% Initialize variables to ensure they exist even if extraction fails partway
pitchRate_data = [];
time_vector_pitchRate = [];
sinChirp_data = [];

disp("Attempting to extract signals from Simulink.SimulationOutput object 'out'...");

if isa(out, 'Simulink.SimulationOutput')
    try
        signal_pitchRate_obj = get(out, 'pitchRate');
        if ~isempty(signal_pitchRate_obj)
            if isprop(signal_pitchRate_obj, 'Values') && isa(signal_pitchRate_obj.Values, 'timeseries')
                pitchRate_ts = signal_pitchRate_obj.Values;
                pitchRate_data = pitchRate_ts.Data;
                time_vector_pitchRate = pitchRate_ts.Time;
                disp('Pitch rate data extracted successfully from out.pitchRate.Values.');
            elseif isprop(signal_pitchRate_obj, 'Data') && isprop(signal_pitchRate_obj, 'Time') % Fallback
                pitchRate_data = signal_pitchRate_obj.Data;
                time_vector_pitchRate = signal_pitchRate_obj.Time;
                disp('Pitch rate data extracted successfully from out.pitchRate.Data/Time (direct properties).');
            else
                warning('Could not extract Data/Time from "out.pitchRate". Structure unexpected.');
                disp('Details of out.pitchRate:'); disp(signal_pitchRate_obj);
                dataError = true;
            end
        else
            warning('"pitchRate" signal not found in Simulink.SimulationOutput object "out".');
            dataError = true;
        end
    catch ME_extract_pr
        warning('Error extracting pitchRate: %s', ME_extract_pr.message);
        dataError = true;
    end

    try
        signal_sinChirp_obj = get(out, 'sinChirp');
        if ~isempty(signal_sinChirp_obj)
            if isprop(signal_sinChirp_obj, 'Values') && isa(signal_sinChirp_obj.Values, 'timeseries')
                sinChirp_ts = signal_sinChirp_obj.Values;
                sinChirp_data = sinChirp_ts.Data;
                disp('Chirp signal data extracted successfully from out.sinChirp.Values.');
            elseif isprop(signal_sinChirp_obj, 'Data') && isprop(signal_sinChirp_obj, 'Time') % Fallback
                sinChirp_data = signal_sinChirp_obj.Data;
                disp('Chirp signal data extracted successfully from out.sinChirp.Data/Time (direct properties).');
            else
                warning('Could not extract Data/Time from "out.sinChirp". Structure unexpected.');
                disp('Details of out.sinChirp:'); disp(signal_sinChirp_obj);
                dataError = true;
            end
        else
            warning('"sinChirp" signal not found in Simulink.SimulationOutput object "out".');
            dataError = true;
        end
    catch ME_extract_sc
        warning('Error extracting sinChirp: %s', ME_extract_sc.message);
        dataError = true;
    end

else
    warning('Output "out" is not a Simulink.SimulationOutput object. Attempting direct field access (legacy).');
    if isfield(out, 'pitchRate') && isprop(out.pitchRate, 'Data') && isprop(out.pitchRate, 'Time')
        pitchRate_data = out.pitchRate.Data;
        time_vector_pitchRate = out.pitchRate.Time;
        disp('Pitch rate data extracted (direct field).');
    else
        warning('Could not find "out.pitchRate.Data" or "out.pitchRate.Time" (direct field).');
        dataError = true;
    end

    if isfield(out, 'sinChirp') && isprop(out.sinChirp, 'Data')
        sinChirp_data = out.sinChirp.Data;
        disp('Chirp signal data extracted (direct field).');
    else
        warning('Could not find "out.sinChirp.Data" (direct field).');
        dataError = true;
    end
end

% After attempting extraction, check for errors and the existence of essential data
if dataError || isempty(pitchRate_data) || isempty(sinChirp_data) || isempty(time_vector_pitchRate)
    if isa(out, 'Simulink.SimulationOutput') && ~dataError % If no specific error, but data is still empty
        disp("Signal objects might be empty. Available signals in 'out' (Simulink.SimulationOutput object):");
        disp(get(out));
    elseif ~isa(out, 'Simulink.SimulationOutput') && ~dataError
        disp("Contents of 'out' (legacy structure):");
        disp(out);
    end
    error('Failed to extract one or both required signals (pitchRate_data, sinChirp_data, or time_vector_pitchRate). Please check Simulink model configuration, signal logging names/methods, and ensure signals are not empty.');
end

time_data = time_vector_pitchRate;

% --- Create an iddata Object ---
pitchRate_data = pitchRate_data(:);
sinChirp_data = sinChirp_data(:);

if length(time_data) > 1
    Ts = mean(diff(time_data));
else
    Ts = 0;
    warning('Only one time sample found. Ts set to 0.');
end
disp(['Sample time (Ts) detected: ', num2str(Ts), ' s']);

ident_data = iddata(pitchRate_data, sinChirp_data, Ts);
ident_data.Name = 'RocketPitchRateModel';
ident_data.InputName = {'ChirpInput'};
ident_data.InputUnit = {'CommandUnits'}; % e.g., 'Volts', 'deg' - UPDATE THIS
ident_data.OutputName = {'PitchRate'};
ident_data.OutputUnit = {'rad/s'}; % Or 'deg/s' - UPDATE THIS
ident_data.Tstart = time_data(1);
ident_data.Notes = 'Data from Simulink STEVE_Simulation for pitch rate model identification using a chirp input.';
disp('iddata object created.');

% --- Save Data to .mat File ---
resultsFolder = fullfile(projectRoot, 'data', 'identification_data');
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
    disp(['Created results directory: ', resultsFolder]);
end
mat_filename = fullfile(resultsFolder, 'pitch_system_id_data.mat');

disp(['Saving identification data to: ', mat_filename]);
save(mat_filename, 'ident_data', 'pitchRate_data', 'sinChirp_data', 'time_data', 'Ts');
disp('Data successfully saved to .mat file.');

% --- Optional: Quick Plot for Verification ---
figure('Name', 'Exported System ID Data Verification');
subplot(2,1,1);
plot(ident_data.Time, ident_data.InputData);
title(['Input Signal: ', ident_data.InputName{1}]);
xlabel('Time (s)');
ylabel(ident_data.InputUnit{1});
grid on;

subplot(2,1,2);
plot(ident_data.Time, ident_data.OutputData);
title(['Output Signal: ', ident_data.OutputName{1}]);
xlabel('Time (s)');
ylabel(ident_data.OutputUnit{1});
grid on;

disp('--- Script Finished ---');
