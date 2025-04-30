% --- Script: Pitch Rate and Tilt Angle Sweep ---
clear; clc; close all; % Start fresh for a sweep script
originalPath = path; % Store original path

% --- Configuration ---
useParallel = true; % Set to true for parsim, false for sequential sim
skip_initial_points = 5; % Number of initial data points to skip for Max AoA calculation
filter_aoa_start_time = 5.0; % Only consider AoA after this time for max calculation

% --- Initialize Timing Variables ---
parallelTime = [];
sequentialTime = [];

% --- Setup: Robust Path Configuration ---
fprintf('Setting up paths...\n');
scriptPath = mfilename('fullpath');
[scriptDir, ~, ~] = fileparts(scriptPath); % Gets '/.../STEVE_Simulator/src/scripts'

% Navigate up to the project root directory (assuming scripts is in src)
projectRoot = fullfile(scriptDir, '..', '..'); % Should be '/.../STEVE_Simulator'

fprintf('Script directory: %s\n', scriptDir);
fprintf('Project root identified as: %s\n', projectRoot);

% Define key directories relative to the project root
modelDir = fullfile(projectRoot, 'src', 'models');
scriptDir = fullfile(projectRoot, 'src', 'scripts'); % Where init/design scripts are assumed to be
resultsDir = fullfile(projectRoot, 'data', 'results');

% Add necessary directories to the MATLAB path
fprintf('Adding model directory to path: %s\n', modelDir);
addpath(modelDir);
fprintf('Adding script directory to path: %s\n', scriptDir);
addpath(scriptDir); % Ensures initialize_parameters, ControlSystemDesign are findable

% Create results directory if it doesn't exist
if ~exist(resultsDir, 'dir')
    fprintf('Creating results directory: %s\n', resultsDir);
    mkdir(resultsDir);
end

% --- Locate and Load Model ---
modelName = 'STEVE_Simulation';
modelPath = fullfile(modelDir, [modelName, '.slx']);
fprintf('Looking for model at: %s\n', modelPath);
if ~exist(modelPath, 'file')
    error('Cannot find Simulink model at: %s. Check modelDir path.', modelPath);
end
% Load the model into memory once before the loop for efficiency
load_system(modelName);
fprintf('Model %s loaded.\n', modelName);

% --- Sweep Configuration ---
% Define pitch-rate and tilt angle sweeps
pitchRates   = 0.0:0.05:0.3;            % Initial pitch rates to test [deg/s]
tiltAngles   = [75, 80, 85];        % Initial tilt angles to test [deg]

nRates       = numel(pitchRates);
nTilts       = numel(tiltAngles);
totalRuns    = nRates * nTilts; % Total number of simulations

% Preallocate storage
runPitchRates    = zeros(totalRuns, 1); % Store the pitch rate for each run
runTiltAngles    = zeros(totalRuns, 1); % Store the tilt angle for each run
nozzleHist       = cell(totalRuns,1);
thetaHist        = cell(totalRuns,1);
thetaCmdHist     = cell(totalRuns,1);
maxHorizDistHist = zeros(totalRuns, 1);
maxAltitudeHist  = zeros(totalRuns, 1);
maxAoAHist       = zeros(totalRuns, 1);
aoaHist          = cell(totalRuns, 1);
timeVec          = []; % To store the time vector (assumed constant across runs)
tRunHist         = cell(totalRuns, 1);
simOutArray      = []; % Initialize simOutArray for parsim results

% --- Simulation Execution ---
if useParallel
    % --- Simulation Setup for parsim ---
    fprintf('\nSetting up parallel simulations for %d runs (across %d tilt angles and %d pitch rates)...\n', totalRuns, nTilts, nRates);
    in(1:totalRuns) = Simulink.SimulationInput(modelName);

    runIndex = 0; % Initialize run counter
    for i_tilt = 1:nTilts
        currentTiltAngle = tiltAngles(i_tilt);
        for j_rate = 1:nRates
            runIndex = runIndex + 1; % Increment run index
            currentPitchRate = pitchRates(j_rate);

            fprintf('Configuring simulation input for run index %d (Tilt=%.0f deg, PR=%.2f deg/s)...\n', runIndex, currentTiltAngle, currentPitchRate);

            % Store parameters for this run
            runTiltAngles(runIndex) = currentTiltAngle;
            runPitchRates(runIndex) = currentPitchRate;

            % --- Run Initialization for this specific run ---
            % (Initialize variables to empty for safety in each iteration)
            localInitial = []; localActuators = []; localRocketAero = []; localPrelookupData = [];
            localMach_Breakpoints = []; localAlpha_Breakpoints = []; localRef_Area = [];
            localWind = []; localSim = []; localAeroData = []; localRocketAeroPhysical = [];
            local_rocket_length = []; local_timeStep = [];
            try
                % Capture all necessary outputs from initialization
                [tempInitial, localActuators, localRocketAero, localPrelookupData, localMach_Breakpoints, localAlpha_Breakpoints, localRef_Area, localWind, localSim, localAeroData, localRocketAeroPhysical, local_rocket_length, local_timeStep] = initialize_parameters_func();
                % Run control system design
                localInitial = ControlSystemDesign_func(tempInitial);
            catch ME_init
                fprintf('ERROR during initialization/design for run index %d setup (Tilt=%.0f, PR=%.2f):\n', runIndex, currentTiltAngle, currentPitchRate);
                fprintf('Error message: %s\n', ME_init.message);
                rethrow(ME_init);
            end

            % --- Verify and Update the specific parameters for this run ---
            if isempty(localInitial) || ~isfield(localInitial,'Conditions') || ~isfield(localInitial.Conditions, 'pitchRate') || ~isfield(localInitial.Conditions, 'tiltAngle')
                error('Variable ''Initial.Conditions.pitchRate'' or ''Initial.Conditions.tiltAngle'' not found after running initialization/design functions for run index %d. Check those functions.', runIndex);
            end
            localInitial.Conditions.tiltAngle = deg2rad(currentTiltAngle); % Override tilt angle (convert to radians)
            localInitial.Conditions.pitchRate = currentPitchRate;         % Override pitch rate

            % --- Verify other required variables exist before adding ---
            % (Add checks similar to the original script if needed, referencing runIndex)
            if ~isfield(localInitial, 'Control') || ...
               ~isfield(localInitial.Control, 'outer_Kp') % Add other checks as before
                error('Required control gains or other variables not found for run index %d. Check initialization/design functions.', runIndex);
            end
             % Add checks for Wind, Sim, AeroData, etc. as in the original script

            % --- Assign variables to the SimulationInput object ---
            in(runIndex) = in(runIndex).setVariable('Initial', localInitial);
            in(runIndex) = in(runIndex).setVariable('Actuators', localActuators);
            in(runIndex) = in(runIndex).setVariable('RocketAero', localRocketAero);
            in(runIndex) = in(runIndex).setVariable('PrelookupData', localPrelookupData);
            in(runIndex) = in(runIndex).setVariable('Mach_Breakpoints', localMach_Breakpoints);
            in(runIndex) = in(runIndex).setVariable('Alpha_Breakpoints', localAlpha_Breakpoints);
            in(runIndex) = in(runIndex).setVariable('Ref_Area', localRef_Area);
            in(runIndex) = in(runIndex).setVariable('Wind', localWind);
            in(runIndex) = in(runIndex).setVariable('outer_Kp', localInitial.Control.outer_Kp);
            in(runIndex) = in(runIndex).setVariable('outer_Ki', localInitial.Control.outer_Ki);
            in(runIndex) = in(runIndex).setVariable('outer_Kd', localInitial.Control.outer_Kd);
            in(runIndex) = in(runIndex).setVariable('inner_Kp', localInitial.Control.inner_Kp);
            in(runIndex) = in(runIndex).setVariable('inner_Ki', localInitial.Control.inner_Ki);
            in(runIndex) = in(runIndex).setVariable('inner_Kd', localInitial.Control.inner_Kd);
            in(runIndex) = in(runIndex).setVariable('Sim', localSim);
            in(runIndex) = in(runIndex).setVariable('AeroData', localAeroData);
            in(runIndex) = in(runIndex).setVariable('RocketAeroPhysical', localRocketAeroPhysical);
            in(runIndex) = in(runIndex).setVariable('rocket_length', local_rocket_length);
            in(runIndex) = in(runIndex).setVariable('timeStep', local_timeStep);

            % Set StopTime using model parameter for clarity with parsim
            in(runIndex) = in(runIndex).setModelParameter('StopTime', num2str(localSim.Time));

        end % End pitch rate loop
    end % End tilt angle loop
    fprintf('Simulation inputs configured.\n');

    % --- Run Simulations in Parallel using parsim ---
    fprintf('\nStarting parallel simulations using parsim...\n');
    parsimTic = tic; % Start timer for parsim
    try
        simOutArray = parsim(in, 'ShowProgress', 'on');
    catch ME_parsim
        fprintf('\n--- PARSIM Error ---\n');
        fprintf('Parallel simulation failed.\nError message: %s\n', ME_parsim.message);
        rethrow(ME_parsim);
    end
    parallelTime = toc(parsimTic); % Stop timer for parsim
    fprintf('...Parallel simulations complete.\n');
    fprintf('\nPARSIM execution time: %.2f seconds\n', parallelTime);

    % --- Check for Simulation Errors BEFORE Data Extraction ---
    errorOccurred = false;
    firstErrorMsg = '';
    firstErrorRunIndex = -1;
    for k = 1:totalRuns
        if ~isempty(simOutArray(k).ErrorMessage)
            errorOccurred = true;
            firstErrorMsg = simOutArray(k).ErrorMessage;
            firstErrorRunIndex = k;
            break; % Stop after finding the first error
        end
    end

    if errorOccurred
        fprintf('\n--- !!! PARSIM Simulation Error Detected !!! ---\n');
        fprintf('Error occurred in at least run index %d (Tilt=%.0f deg, PR=%.2f deg/s).\n', firstErrorRunIndex, runTiltAngles(firstErrorRunIndex), runPitchRates(firstErrorRunIndex));
        fprintf('Error Message:\n%s\n', firstErrorMsg);
        fprintf('-------------------------------------------------\n');
        error('Stopping script due to simulation errors during parsim.');
    end

    % --- Extract Data from parsim Results ---
    fprintf('Extracting data from simulation results...\n');
    for i = 1:totalRuns % Loop through all runs using the single index
        currentSimOut = simOutArray(i);
        currentTiltAngle = runTiltAngles(i); % Get parameters for this run
        currentPitchRate = runPitchRates(i);
        try
            tRun_local           = currentSimOut.tout;
            nRun_local           = currentSimOut.NozzleAngle.Data * 180/pi;
            thRun_local          = currentSimOut.theta.Data       * 180/pi;
            thCmdRun_local       = currentSimOut.thetaCmd.Data    * 180/pi;
            xeRun_local          = currentSimOut.Xe.Data;
            alphaRun_local_rad   = currentSimOut.alpha.Data; % Extract AoA in radians

            % Convert AoA to degrees
            alphaRun_local_deg = alphaRun_local_rad * 180/pi;

            % Filter AoA by time and calculate max absolute AoA
            time_inds = tRun_local >= filter_aoa_start_time;
            if any(time_inds)
                currentMaxAbsAoA_deg = max(abs(alphaRun_local_deg(time_inds)));
            else
                currentMaxAbsAoA_deg = NaN; % No data points after filter time
            end

            % Calculate Max Horizontal Distance and Altitude
            if isempty(xeRun_local) || size(xeRun_local, 2) < 3
                 currentMaxHorizDist_km = NaN;
                 currentMaxAltitude_km = NaN;
            else
                 % Horizontal distance (assuming x is North, y is East)
                 horizDist_m = sqrt(xeRun_local(:,1).^2 + xeRun_local(:,2).^2);
                 currentMaxHorizDist_km = max(horizDist_m) / 1000;
                 % Altitude (assuming z is Down, so take negative)
                 altitude_m = -xeRun_local(:,3);
                 currentMaxAltitude_km = max(altitude_m) / 1000;
            end

        catch ME_extract
            fprintf('\n--- Data Extraction Error (Parsim Run %d, Tilt=%.0f, PR=%.2f) ---\n', i, currentTiltAngle, currentPitchRate);
            fprintf('Error: %s\n', ME_extract.message);
            % Assign NaN to results for this failed run to maintain array sizes
            tRunHist{i}         = []; % Mark as failed extraction
            nozzleHist{i}       = [];
            thetaHist{i}        = [];
            thetaCmdHist{i}     = [];
            aoaHist{i}          = [];
            maxHorizDistHist(i) = NaN;
            maxAltitudeHist(i)  = NaN;
            maxAoAHist(i)       = NaN;
            continue; % Skip to the next run
        end

        % --- Store Results ---
        tRunHist{i}         = tRun_local;
        nozzleHist{i}       = nRun_local;
        thetaHist{i}        = thRun_local;
        thetaCmdHist{i}     = thCmdRun_local;
        aoaHist{i}          = alphaRun_local_deg; % Store full AoA history (degrees)
        maxHorizDistHist(i) = currentMaxHorizDist_km;
        maxAltitudeHist(i)  = currentMaxAltitude_km;
        maxAoAHist(i)       = currentMaxAbsAoA_deg; % Store max AoA (calculated using time filter)
    end
    fprintf('Data extraction complete.\n');

else % --- Sequential Simulation using sim ---
    fprintf('\nStarting sequential simulations using sim...\n');
    sequentialTic = tic; % Start timer for sequential loop

    runIndex = 0; % Initialize run counter
    for i_tilt = 1:nTilts
        currentTiltAngle = tiltAngles(i_tilt);
        for j_rate = 1:nRates
            runIndex = runIndex + 1; % Increment run index
            currentPitchRate = pitchRates(j_rate);

            fprintf('Running sequential simulation for run index %d (Tilt=%.0f deg, PR=%.2f deg/s)...\n', runIndex, currentTiltAngle, currentPitchRate);

             % Store parameters for this run
            runTiltAngles(runIndex) = currentTiltAngle;
            runPitchRates(runIndex) = currentPitchRate;

            % --- Run Initialization for this specific run ---
            localInitial = []; localActuators = []; localRocketAero = []; localPrelookupData = [];
            localMach_Breakpoints = []; localAlpha_Breakpoints = []; localRef_Area = [];
            localWind = []; localSim = []; localAeroData = []; localRocketAeroPhysical = [];
            local_rocket_length = []; local_timeStep = [];
            try
                [tempInitial, localActuators, localRocketAero, localPrelookupData, localMach_Breakpoints, localAlpha_Breakpoints, localRef_Area, localWind, localSim, localAeroData, localRocketAeroPhysical, local_rocket_length, local_timeStep] = initialize_parameters_func();
                localInitial = ControlSystemDesign_func(tempInitial);
                localInitial.Conditions.tiltAngle = deg2rad(currentTiltAngle); % Override tilt angle
                localInitial.Conditions.pitchRate = currentPitchRate;         % Override pitch rate
            catch ME_init_seq
                fprintf('ERROR during initialization/design for sequential run index %d (Tilt=%.0f, PR=%.2f):\n', runIndex, currentTiltAngle, currentPitchRate);
                fprintf('Error message: %s\n', ME_init_seq.message);
                 % Assign NaN and continue
                tRunHist{runIndex} = []; nozzleHist{runIndex} = []; thetaHist{runIndex} = []; thetaCmdHist{runIndex} = []; aoaHist{runIndex} = [];
                maxHorizDistHist(runIndex) = NaN; maxAltitudeHist(runIndex) = NaN; maxAoAHist(runIndex) = NaN;
                continue; % Continue to next iteration
            end

            % --- Assign ALL required variables to base workspace for sim ---
            try
                assignin('base', 'Initial', localInitial);
                assignin('base', 'Actuators', localActuators);
                assignin('base', 'RocketAero', localRocketAero);
                assignin('base', 'PrelookupData', localPrelookupData);
                assignin('base', 'Mach_Breakpoints', localMach_Breakpoints);
                assignin('base', 'Alpha_Breakpoints', localAlpha_Breakpoints);
                assignin('base', 'Ref_Area', localRef_Area);
                assignin('base', 'Wind', localWind);
                assignin('base', 'outer_Kp', localInitial.Control.outer_Kp);
                assignin('base', 'outer_Ki', localInitial.Control.outer_Ki);
                assignin('base', 'outer_Kd', localInitial.Control.outer_Kd);
                assignin('base', 'inner_Kp', localInitial.Control.inner_Kp);
                assignin('base', 'inner_Ki', localInitial.Control.inner_Ki);
                assignin('base', 'inner_Kd', localInitial.Control.inner_Kd);
                assignin('base', 'Sim', localSim);
                assignin('base', 'AeroData', localAeroData);
                assignin('base', 'RocketAeroPhysical', localRocketAeroPhysical);
                assignin('base', 'rocket_length', local_rocket_length);
                assignin('base', 'timeStep', local_timeStep);
            catch ME_assign
                fprintf('ERROR assigning variables to base workspace for sequential run index %d (Tilt=%.0f, PR=%.2f):\n', runIndex, currentTiltAngle, currentPitchRate);
                fprintf('Error message: %s\n', ME_assign.message);
                 % Assign NaN and continue
                tRunHist{runIndex} = []; nozzleHist{runIndex} = []; thetaHist{runIndex} = []; thetaCmdHist{runIndex} = []; aoaHist{runIndex} = [];
                maxHorizDistHist(runIndex) = NaN; maxAltitudeHist(runIndex) = NaN; maxAoAHist(runIndex) = NaN;
                continue; % Continue to next iteration
            end

            % --- Run the simulation using sim ---
            simOut = []; % Clear previous simOut
            try
                simOut = sim(modelName, 'StopTime', num2str(localSim.Time));
            catch ME_sim
                fprintf('ERROR during simulation for sequential run index %d (Tilt=%.0f, PR=%.2f):\n', runIndex, currentTiltAngle, currentPitchRate);
                fprintf('Error message: %s\n', ME_sim.message);
                 % Assign NaN and continue
                tRunHist{runIndex} = []; nozzleHist{runIndex} = []; thetaHist{runIndex} = []; thetaCmdHist{runIndex} = []; aoaHist{runIndex} = [];
                maxHorizDistHist(runIndex) = NaN; maxAltitudeHist(runIndex) = NaN; maxAoAHist(runIndex) = NaN;
                continue; % Continue to next iteration
            end

            % --- Extract and store results directly ---
            try
                tRun_local           = simOut.tout;
                nRun_local           = simOut.NozzleAngle.Data * 180/pi;
                thRun_local          = simOut.theta.Data       * 180/pi;
                thCmdRun_local       = simOut.thetaCmd.Data    * 180/pi;
                xeRun_local          = simOut.Xe.Data;
                alphaRun_local_rad   = simOut.alpha.Data;
                alphaRun_local_deg   = alphaRun_local_rad * 180/pi;

                % Filter AoA by time
                time_inds = tRun_local >= filter_aoa_start_time;
                if any(time_inds)
                    currentMaxAbsAoA_deg = max(abs(alphaRun_local_deg(time_inds)));
                else
                    currentMaxAbsAoA_deg = NaN;
                end

                % Calculate Max Distances
                if isempty(xeRun_local) || size(xeRun_local, 2) < 3
                     currentMaxHorizDist_km = NaN;
                     currentMaxAltitude_km = NaN;
                else
                     horizDist_m = sqrt(xeRun_local(:,1).^2 + xeRun_local(:,2).^2);
                     currentMaxHorizDist_km = max(horizDist_m) / 1000;
                     altitude_m = -xeRun_local(:,3);
                     currentMaxAltitude_km = max(altitude_m) / 1000;
                end

                % Store
                tRunHist{runIndex}         = tRun_local;
                nozzleHist{runIndex}       = nRun_local;
                thetaHist{runIndex}        = thRun_local;
                thetaCmdHist{runIndex}     = thCmdRun_local;
                aoaHist{runIndex}          = alphaRun_local_deg;
                maxHorizDistHist(runIndex) = currentMaxHorizDist_km;
                maxAltitudeHist(runIndex)  = currentMaxAltitude_km;
                maxAoAHist(runIndex)       = currentMaxAbsAoA_deg;

            catch ME_extract_seq
                fprintf('\n--- Data Extraction Error (Sequential Run %d, Tilt=%.0f, PR=%.2f) ---\n', runIndex, currentTiltAngle, currentPitchRate);
                fprintf('Error: %s\n', ME_extract_seq.message);
                % Assign NaN
                tRunHist{runIndex} = []; nozzleHist{runIndex} = []; thetaHist{runIndex} = []; thetaCmdHist{runIndex} = []; aoaHist{runIndex} = [];
                maxHorizDistHist(runIndex) = NaN; maxAltitudeHist(runIndex) = NaN; maxAoAHist(runIndex) = NaN;
                % Continue loop
            end

        end % End pitch rate loop
    end % End tilt angle loop
    sequentialTime = toc(sequentialTic); % Stop timer for sequential loop
    fprintf('...Sequential simulations complete.\n');
    fprintf('\nSEQUENTIAL execution time: %.2f seconds\n', sequentialTime);

end % End if useParallel

% --- Post-Processing: Time Vector Alignment and Interpolation ---
fprintf('Aligning time vectors and interpolating results...\n');

% Find the first valid run to use as reference
firstValidRun = find(~cellfun('isempty', tRunHist), 1);
if isempty(firstValidRun)
    warning('No valid simulation runs completed. Cannot proceed with post-processing or saving.');
    path(originalPath); % Restore original path
    return;
end
timeVec = tRunHist{firstValidRun};
nTimeRef = length(timeVec);
fprintf('Using time vector from run %d as reference.\n', firstValidRun);

for i = 1:totalRuns
    % Skip runs that failed extraction (have empty time vectors)
    if isempty(tRunHist{i})
        fprintf('Skipping interpolation for run index %d (failed extraction).\n', i);
        % Ensure data is NaN for saving consistency
        nozzleHist{i} = NaN(nTimeRef, 1);
        thetaHist{i} = NaN(nTimeRef, 1);
        thetaCmdHist{i} = NaN(nTimeRef, 1);
        aoaHist{i} = NaN(nTimeRef, 1);
        continue;
    end

    % Check if interpolation is needed for this run
    if length(tRunHist{i}) ~= nTimeRef || any(abs(tRunHist{i} - timeVec) > 1e-9 * max(abs(timeVec)))
        fprintf('Interpolating results for run index %d onto common time vector.\n', i);
        % Interpolate results if time vectors differ
        try
            nozzleHist{i}   = interp1(tRunHist{i}, nozzleHist{i},   timeVec, 'linear', 'extrap');
            thetaHist{i}    = interp1(tRunHist{i}, thetaHist{i},    timeVec, 'linear', 'extrap');
            thetaCmdHist{i} = interp1(tRunHist{i}, thetaCmdHist{i}, timeVec, 'linear', 'extrap');
            aoaHist{i}      = interp1(tRunHist{i}, aoaHist{i},      timeVec, 'linear', 'extrap');
        catch ME_interp
            warning('Setting data for run %d to NaN due to interpolation error: %s', i, ME_interp.message);
            nozzleHist{i} = NaN(nTimeRef, 1);
            thetaHist{i} = NaN(nTimeRef, 1);
            thetaCmdHist{i} = NaN(nTimeRef, 1);
            aoaHist{i} = NaN(nTimeRef, 1);
        end
    end
end
fprintf('Interpolation complete.\n');

% --- Display Timing Results ---
fprintf('\n--- Timing Comparison ---\n');
if useParallel && ~isempty(parallelTime)
    fprintf('Parallel execution time: %.2f seconds\n', parallelTime);
end
if ~useParallel && ~isempty(sequentialTime)
    fprintf('Sequential execution time: %.2f seconds\n', sequentialTime);
elseif useParallel && exist('sequentialTime', 'var') && ~isempty(sequentialTime) % If parallel was used but sequential was also run for comparison
     fprintf('Sequential execution time (for comparison): %.2f seconds\n', sequentialTime);
end

% --- Save Results ---
% Use a fixed filename, now indicating 2D sweep
resultsFilename = fullfile(resultsDir, 'tilt_pitch_rate_sweep_results.mat');
fprintf('\nSaving results to: %s (overwriting if exists)\n', resultsFilename);
try
    % Save extracted/interpolated data and key parameters
    save(resultsFilename, ...
         'pitchRates', 'tiltAngles', 'totalRuns', 'nRates', 'nTilts', ... % Sweep definitions
         'runPitchRates', 'runTiltAngles', ... % Parameters for each run
         'timeVec', 'tRunHist', ...            % Time vectors
         'nozzleHist', 'thetaHist', 'thetaCmdHist', 'aoaHist', ... % Time histories
         'maxHorizDistHist', 'maxAltitudeHist', 'maxAoAHist', ... % Performance metrics
         'parallelTime', 'sequentialTime', 'useParallel', ... % Timing and config
         '-v7.3'); % Use v7.3 format for potentially large data

    fprintf('Results saved successfully.\n');
catch ME_save
    fprintf('Error saving results: %s\n', ME_save.message);
end

fprintf('\nPitch rate and tilt angle sweep script finished.\n');

% Restore original path
path(originalPath);
