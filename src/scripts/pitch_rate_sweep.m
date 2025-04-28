% --- Script: Pitch Rate Sweep ---
clear; clc; close all; % Start fresh for a sweep script

% --- Configuration ---
useParallel = true; % Set to true for parsim, false for sequential sim

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
% Define pitch-rate sweep and preallocate storage
pitchRates   = 0:0.25:1;            % Initial pitch rates to test [deg/s]
nRuns        = numel(pitchRates);
totalRuns    = nRuns; % Store the total number of runs safely
nozzleHist   = cell(totalRuns,1);      % Use totalRuns for allocation
thetaHist    = cell(totalRuns,1);      % Use totalRuns for allocation
thetaCmdHist = cell(totalRuns,1);      % Use totalRuns for allocation
legendEntriesPlot1 = cell(totalRuns, 1); % Preallocate legend strings for Plot 1
legendEntriesPlot2 = cell(totalRuns, 1); % Preallocate legend strings for Plot 2
maxHorizDistHist = zeros(totalRuns, 1); % Preallocate storage for max horizontal distance
maxAltitudeHist  = zeros(totalRuns, 1); % Preallocate storage for max altitude
timeVec      = [];                 % To store the time vector (assumed constant across runs)
tRunHist     = cell(totalRuns, 1); % Preallocate cell array to store individual time vectors for each run

% --- Simulation Execution ---
if useParallel
    % --- Simulation Setup for parsim ---
    fprintf('\nSetting up parallel simulations for %d runs...\n', totalRuns);
    in(1:totalRuns) = Simulink.SimulationInput(modelName);

    % Configure each simulation input object
    for i = 1:totalRuns
        currentPitchRate = pitchRates(i);
        fprintf('Configuring simulation input for run index %d (PR=%.1f deg/s)...\n', i, currentPitchRate);

        % --- Run Initialization for this specific run ---
        localInitial = [];
        localActuators = [];
        localRocketAero = [];
        localPrelookupData = [];
        localMach_Breakpoints = [];
        localAlpha_Breakpoints = [];
        localRef_Area = [];
        localWind = []; 
        localSim = []; 
        localAeroData = []; 
        localRocketAeroPhysical = []; 
        local_rocket_length = []; 
        local_timeStep = []; 
        try
            % Capture all necessary outputs
            [tempInitial, localActuators, localRocketAero, localPrelookupData, localMach_Breakpoints, localAlpha_Breakpoints, localRef_Area, localWind, localSim, localAeroData, localRocketAeroPhysical, local_rocket_length, local_timeStep] = initialize_parameters_func(); 
            localInitial = ControlSystemDesign_func(tempInitial); 
        catch ME_init
            fprintf('ERROR during initialization/design for run index %d setup (PR=%.1f):\n', i, currentPitchRate);
            fprintf('Error message: %s\n', ME_init.message);
            rethrow(ME_init); 
        end

        % --- Verify and Update the specific parameter for this run ---
        if isempty(localInitial) || ~isfield(localInitial,'Conditions') || ~isfield(localInitial.Conditions, 'pitchRate')
            error('Variable ''Initial.Conditions.pitchRate'' not found after running initialization/design functions for run index %d. Check those functions.', i);
        end
        localInitial.Conditions.pitchRate = currentPitchRate; % Override the specific value

        % --- Verify required variables exist before adding --- 
        % (Checks for PID gains, Wind, Sim, AeroData, RocketAeroPhysical, rocket_length, timeStep)
        if ~isfield(localInitial, 'Control') || ...
           ~isfield(localInitial.Control, 'outer_Kp') || ~isfield(localInitial.Control, 'outer_Ki') || ~isfield(localInitial.Control, 'outer_Kd') || ...
           ~isfield(localInitial.Control, 'inner_Kp') || ~isfield(localInitial.Control, 'inner_Ki') || ~isfield(localInitial.Control, 'inner_Kd')
            error('PID gains not found in Initial.Control structure for run index %d. Check ControlSystemDesign_func.', i);
        end
        if isempty(localWind) || ~isstruct(localWind)
            error('Wind structure not returned correctly from initialize_parameters_func for run index %d.', i);
        end
        if isempty(localSim) || ~isstruct(localSim) || ~isfield(localSim, 'Time')
            error('Sim structure (with Time field) not returned correctly from initialize_parameters_func for run index %d.', i);
        end
        if isempty(localAeroData) || ~isstruct(localAeroData) || ~isfield(localAeroData, 'Tables') || ~isfield(localAeroData.Tables, 'CN')
            error('AeroData structure (with Tables.CN field) not returned correctly from initialize_parameters_func for run index %d.', i);
        end
        if isempty(localRocketAeroPhysical) || ~isstruct(localRocketAeroPhysical) || ~isfield(localRocketAeroPhysical, 'Diameter')
            error('RocketAeroPhysical structure (with Diameter field) not returned correctly from initialize_parameters_func for run index %d.', i);
        end
        if isempty(local_rocket_length) || ~isscalar(local_rocket_length)
            error('rocket_length not returned correctly from initialize_parameters_func for run index %d.', i);
        end
        if isempty(local_timeStep) || ~isscalar(local_timeStep)
            error('timeStep not returned correctly from initialize_parameters_func for run index %d.', i);
        end

        % --- Assign variables to the SimulationInput object ---
        in(i) = in(i).setVariable('Initial', localInitial);
        in(i) = in(i).setVariable('Actuators', localActuators);
        in(i) = in(i).setVariable('RocketAero', localRocketAero); 
        in(i) = in(i).setVariable('PrelookupData', localPrelookupData);
        in(i) = in(i).setVariable('Mach_Breakpoints', localMach_Breakpoints);
        in(i) = in(i).setVariable('Alpha_Breakpoints', localAlpha_Breakpoints);
        in(i) = in(i).setVariable('Ref_Area', localRef_Area);
        in(i) = in(i).setVariable('Wind', localWind); 
        in(i) = in(i).setVariable('outer_Kp', localInitial.Control.outer_Kp);
        in(i) = in(i).setVariable('outer_Ki', localInitial.Control.outer_Ki);
        in(i) = in(i).setVariable('outer_Kd', localInitial.Control.outer_Kd);
        in(i) = in(i).setVariable('inner_Kp', localInitial.Control.inner_Kp);
        in(i) = in(i).setVariable('inner_Ki', localInitial.Control.inner_Ki);
        in(i) = in(i).setVariable('inner_Kd', localInitial.Control.inner_Kd);
        in(i) = in(i).setVariable('Sim', localSim); 
        in(i) = in(i).setVariable('AeroData', localAeroData); 
        in(i) = in(i).setVariable('RocketAeroPhysical', localRocketAeroPhysical); 
        in(i) = in(i).setVariable('rocket_length', local_rocket_length); 
        in(i) = in(i).setVariable('timeStep', local_timeStep); 
        
        % Set StopTime using model parameter for clarity with parsim
        in(i) = in(i).setModelParameter('StopTime', num2str(localSim.Time));

    end
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
    parsimTime = toc(parsimTic); % Stop timer for parsim
    fprintf('...Parallel simulations complete.\n');
    fprintf('\nPARSIM execution time: %.2f seconds\n', parsimTime);

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
        fprintf('Error occurred in at least run index %d (PR=%.1f deg/s).\n', firstErrorRunIndex, pitchRates(firstErrorRunIndex));
        fprintf('Error Message:\n%s\n', firstErrorMsg);
        fprintf('-------------------------------------------------\n');
        error('Stopping script due to simulation errors during parsim.'); 
    end

    % --- Extract Data from parsim Results --- 
    fprintf('Extracting data from simulation results...\n');
    for i = 1:totalRuns
        currentSimOut = simOutArray(i);
        currentPitchRate = pitchRates(i); 
        try
            tRun_local           = currentSimOut.tout; 
            nRun_local           = currentSimOut.NozzleAngle.Data * 180/pi;
            thRun_local          = currentSimOut.theta.Data       * 180/pi;
            thCmdRun_local       = currentSimOut.thetaCmd.Data    * 180/pi;
            xeRun_local          = currentSimOut.Xe.Data; 
        catch ME_extract
            fprintf('\n--- Data Extraction Error (Parsim Run %d) ---\n', i);
            fprintf('Failed to extract data after simulation run PR=%.1f deg/s.\n', currentPitchRate);
            fprintf('Error message: %s\n', ME_extract.message);
            tRunHist{i} = []; nozzleHist{i} = []; thetaHist{i} = []; thetaCmdHist{i} = []; 
            maxHorizDistHist(i) = NaN; maxAltitudeHist(i) = NaN;
            warning('Skipping data storage for run %d due to extraction error.', i);
            continue; 
        end

        % --- Calculate & Store Max Distances/Altitude ---
        if isempty(xeRun_local)
            currentMaxHorizDist_km = NaN;
            currentMaxAltitude_km = NaN;
        else
            currentMaxHorizDist_m = max(sqrt(sum(xeRun_local(:,1:2).^2, 2))); 
            currentMaxHorizDist_km = currentMaxHorizDist_m / 1000; 
            currentMaxAltitude_m  = -min(xeRun_local(:,3)); 
            currentMaxAltitude_km = currentMaxAltitude_m / 1000; 
        end
        tRunHist{i}         = tRun_local; 
        nozzleHist{i}       = nRun_local;
        thetaHist{i}        = thRun_local;
        thetaCmdHist{i}     = thCmdRun_local;
        maxHorizDistHist(i) = currentMaxHorizDist_km; 
        maxAltitudeHist(i)  = currentMaxAltitude_km;  
    end 
    fprintf('Data extraction complete.\n');

else % --- Sequential Simulation using sim ---
    fprintf('\nStarting sequential simulations using sim...\n');
    sequentialTic = tic; % Start timer for sequential loop
    
    for i = 1:totalRuns
        currentPitchRate = pitchRates(i);
        fprintf('Running sequential simulation for run index %d (PR=%.1f deg/s)...\n', i, currentPitchRate);

        % --- Run Initialization for this specific run ---
        localInitial = []; localActuators = []; localRocketAero = []; localPrelookupData = [];
        localMach_Breakpoints = []; localAlpha_Breakpoints = []; localRef_Area = [];
        localWind = []; localSim = []; localAeroData = []; localRocketAeroPhysical = [];
        local_rocket_length = []; local_timeStep = [];
        try
            [tempInitial, localActuators, localRocketAero, localPrelookupData, localMach_Breakpoints, localAlpha_Breakpoints, localRef_Area, localWind, localSim, localAeroData, localRocketAeroPhysical, local_rocket_length, local_timeStep] = initialize_parameters_func(); 
            localInitial = ControlSystemDesign_func(tempInitial); 
            localInitial.Conditions.pitchRate = currentPitchRate; % Override pitch rate
        catch ME_init_seq
            fprintf('\n--- INITIALIZATION Error (Sequential Run %d) ---\n', i);
            fprintf('Initialization failed for PR=%.1f deg/s.\nError message: %s\n', currentPitchRate, ME_init_seq.message);
            tRunHist{i} = []; nozzleHist{i} = []; thetaHist{i} = []; thetaCmdHist{i} = []; 
            maxHorizDistHist(i) = NaN; maxAltitudeHist(i) = NaN;
            warning('Skipping sequential run %d due to initialization error.', i);
            continue; % Continue to next iteration
        end

        % --- Assign ALL required variables to base workspace for sim ---
        % (This mimics running the init script before each sim call)
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
             fprintf('\n--- ASSIGNIN Error (Sequential Run %d) ---\n', i);
            fprintf('Failed to assign variables for PR=%.1f deg/s.\nError message: %s\n', currentPitchRate, ME_assign.message);
            tRunHist{i} = []; nozzleHist{i} = []; thetaHist{i} = []; thetaCmdHist{i} = []; 
            maxHorizDistHist(i) = NaN; maxAltitudeHist(i) = NaN;
            warning('Skipping sequential run %d due to assignin error.', i);
            continue; % Continue to next iteration
        end

        % --- Run the simulation using sim --- 
        simOut = []; % Clear previous simOut
        try
            simOut = sim(modelName, 'StopTime', num2str(localSim.Time));
        catch ME_sim
            fprintf('\n--- SIMULATION Error (Sequential Run %d) ---\n', i);
            fprintf('Sequential simulation failed for PR=%.1f deg/s.\nError message: %s\n', currentPitchRate, ME_sim.message);
            tRunHist{i} = []; nozzleHist{i} = []; thetaHist{i} = []; thetaCmdHist{i} = []; 
            maxHorizDistHist(i) = NaN; maxAltitudeHist(i) = NaN;
            warning('Skipping data storage for sequential run %d due to simulation error.', i);
            continue; % Continue to next iteration
        end

        % --- Extract and store results directly --- 
         try
            tRun_local           = simOut.tout; 
            nRun_local           = simOut.NozzleAngle.Data * 180/pi;
            thRun_local          = simOut.theta.Data       * 180/pi;
            thCmdRun_local       = simOut.thetaCmd.Data    * 180/pi;
            xeRun_local          = simOut.Xe.Data; 

            if isempty(xeRun_local)
                currentMaxHorizDist_km = NaN;
                currentMaxAltitude_km = NaN;
            else
                currentMaxHorizDist_m = max(sqrt(sum(xeRun_local(:,1:2).^2, 2))); 
                currentMaxHorizDist_km = currentMaxHorizDist_m / 1000; 
                currentMaxAltitude_m  = -min(xeRun_local(:,3)); 
                currentMaxAltitude_km = currentMaxAltitude_m / 1000; 
            end
            tRunHist{i}         = tRun_local; 
            nozzleHist{i}       = nRun_local;
            thetaHist{i}        = thRun_local;
            thetaCmdHist{i}     = thCmdRun_local;
            maxHorizDistHist(i) = currentMaxHorizDist_km; 
            maxAltitudeHist(i)  = currentMaxAltitude_km;  

         catch ME_extract_seq
            fprintf('\n--- Data Extraction Error (Sequential Run %d) ---\n', i);
            fprintf('Failed to extract data after sequential run PR=%.1f deg/s.\n', currentPitchRate);
            fprintf('Error message: %s\n', ME_extract_seq.message);
             tRunHist{i} = []; nozzleHist{i} = []; thetaHist{i} = []; thetaCmdHist{i} = []; 
             maxHorizDistHist(i) = NaN; maxAltitudeHist(i) = NaN;
             warning('Skipping data storage for sequential run %d due to extraction error.', i);
             continue; % Continue to next iteration
         end

    end % End sequential for loop
    sequentialTime = toc(sequentialTic); % Stop timer for sequential loop
    fprintf('...Sequential simulations complete.\n');
    fprintf('\nSEQUENTIAL execution time: %.2f seconds\n', sequentialTime);

end % End if useParallel

% --- Post-Processing: Time Vector Alignment and Interpolation ---
fprintf('Aligning time vectors and interpolating results...\n');

% Find the first valid run to use as reference
firstValidRun = find(~cellfun('isempty', tRunHist), 1);
if isempty(firstValidRun)
    error('No valid simulation runs completed. Cannot proceed with post-processing.');
end
timeVec = tRunHist{firstValidRun}; 
nTimeRef = length(timeVec);
fprintf('Using time vector from run %d as reference.\n', firstValidRun);

for i = 1:totalRuns
    % Skip runs that failed extraction (have empty time vectors)
    if isempty(tRunHist{i})
        fprintf('Skipping interpolation for run index %d (failed extraction).\n', i);
        % Ensure legend entries are handled for skipped runs if necessary
        legendEntriesPlot1{i} = sprintf('Run %d Failed', i);
        legendEntriesPlot2{i} = sprintf('Run %d Failed', i);
        continue;
    end

    % Check if interpolation is needed for this run
    if length(tRunHist{i}) ~= nTimeRef || any(abs(tRunHist{i} - timeVec) > 1e-9 * max(abs(timeVec)))
        fprintf('Interpolating results for run index %d onto common time vector.\n', i);
        % Interpolate results if time vectors differ
        % Use error handling for interp1 in case of issues
        try
            nozzleHist{i} = interp1(tRunHist{i}, nozzleHist{i}, timeVec, 'linear', 'extrap');
            thetaHist{i}  = interp1(tRunHist{i}, thetaHist{i},  timeVec, 'linear', 'extrap');
            thetaCmdHist{i}= interp1(tRunHist{i}, thetaCmdHist{i},timeVec, 'linear', 'extrap');
        catch ME_interp
            fprintf('ERROR during interpolation for run %d: %s\n', i, ME_interp.message);
            % Handle interpolation error - maybe set data to NaN or skip
            nozzleHist{i} = NaN(size(timeVec));
            thetaHist{i} = NaN(size(timeVec));
            thetaCmdHist{i} = NaN(size(timeVec));
            warning('Setting data for run %d to NaN due to interpolation error.', i);
        end
    end
    
    % --- Generate Legend Entries AFTER the loop ---
    % Ensure pitchRates is available 
    if ~exist('pitchRates', 'var')
        warning('pitchRates variable might have been cleared. Re-defining for legends.');
        pitchRates = 0:0.25:1; % Make sure this matches the definition before the loop
    end
    currentLegendEntry = sprintf('PR=%.1f°/s', pitchRates(i));
    legendEntriesPlot1{i} = currentLegendEntry;
    legendEntriesPlot2{i} = currentLegendEntry;
end
fprintf('Interpolation and legend generation complete.\n');

% --- Plotting Results --- (Uses the now consistent timeVec and interpolated data)
fprintf('Generating plots...\n');

% Plot 1: Nozzle angle vs time for all runs
figure('Name','Nozzle Angle Time Histories');
hold on;
colors1 = lines(totalRuns);
validRunIndices1 = find(~cellfun(@(x) all(isnan(x(:))), nozzleHist)); % Find runs with non-NaN data
legendHandles1 = gobjects(length(validRunIndices1), 1);
legendTexts1 = cell(length(validRunIndices1), 1);
plotIdx = 1;
for i = validRunIndices1(:)' % Iterate through valid runs
    legendHandles1(plotIdx) = plot(timeVec, nozzleHist{i}, 'LineWidth', 1.5, 'Color', colors1(i,:));
    legendTexts1{plotIdx} = legendEntriesPlot1{i};
    plotIdx = plotIdx + 1;
end
hold off;
grid on;
xlabel('Time (s)');
ylabel('Nozzle Deflection (°)');
if ~isempty(legendHandles1)
    legend(legendHandles1, legendTexts1, 'Location','best');
else
    title('Nozzle Angle vs Time (No valid runs to plot)');
end
title('Nozzle Angle vs Time for Various Initial Pitch Rates');
saveas(gcf, fullfile(resultsDir, 'sweep_nozzle_histories.png'));

% Plot 2: θ and θ_cmd vs time for all runs
figure('Name','Pitch Angle vs Command Time Histories');
hold on;
colors2 = lines(totalRuns);
validRunIndices2 = find(~cellfun(@(x) all(isnan(x(:))), thetaHist)); % Find runs with non-NaN theta data
legendHandles2 = gobjects(length(validRunIndices2), 1);
legendTexts2 = cell(length(validRunIndices2), 1);
plotIdx = 1;
for i = validRunIndices2(:)' % Iterate through valid runs
    h_theta = plot(timeVec, thetaHist{i},   '-',  'Color', colors2(i,:), 'LineWidth', 1.5);
    % Check if thetaCmdHist{i} is also valid before plotting
    if i <= length(thetaCmdHist) && ~all(isnan(thetaCmdHist{i}(:)))
        plot(timeVec, thetaCmdHist{i}, '--', 'Color', colors2(i,:), 'LineWidth', 1.0);
    end
    legendHandles2(plotIdx) = h_theta; % Use handle from actual theta plot for legend
    legendTexts2{plotIdx} = legendEntriesPlot2{i};
    plotIdx = plotIdx + 1;
end
hold off;
grid on;
xlabel('Time (s)');
ylabel('Angle (°)');
if ~isempty(legendHandles2)
    legend(legendHandles2, legendTexts2, 'Location','best');
else
     title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', '(No valid runs to plot)'});
end
title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', 'for Various Initial Pitch Rates'});
saveas(gcf, fullfile(resultsDir, 'sweep_pitch_histories.png'));

% Plot 3: Max nozzle deflection vs initial pitch-rate
figure('Name','Max Nozzle Angle vs Initial Pitch Rate');
maxNozzleAbs = cellfun(@(x) max(abs(x)), nozzleHist); % Recalculate in case of NaNs
validRuns3 = ~isnan(maxNozzleAbs);
if ~exist('pitchRates', 'var')
    warning('pitchRates variable was cleared. Reloading for plot 3.');
    pitchRates = 0:0.25:1; 
end
if any(validRuns3)
    plot(pitchRates(validRuns3), maxNozzleAbs(validRuns3), 'o-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'b');
else
    text(0.5, 0.5, 'No valid runs to plot', 'HorizontalAlignment', 'center');
end
grid on;
xlabel('Initial Pitch Rate (°/s)');
ylabel('Maximum Absolute Nozzle Deflection (°)');
title('Max |Nozzle Deflection| vs Initial Pitch Rate');
saveas(gcf, fullfile(resultsDir, 'sweep_max_nozzle_vs_pitchrate.png'));

% Plot 4: Max Altitude & Horizontal Distance vs Initial Pitch Rate
figure('Name','Max Altitude & Horizontal Distance vs Initial Pitch Rate');

if ~exist('pitchRates', 'var')
    warning('pitchRates variable was cleared. Reloading for plot 4.');
    pitchRates = 0:0.25:1; 
end

% Plot Max Altitude on the left y-axis
yyaxis left
validRuns4_alt = ~isnan(maxAltitudeHist);
if any(validRuns4_alt)
    plot(pitchRates(validRuns4_alt), maxAltitudeHist(validRuns4_alt), 'o-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'r', 'DisplayName', 'Max Altitude');
    ylabel('Maximum Altitude (km)');
else
    ylabel('Maximum Altitude (km) - No Data');
end

% Plot Max Horizontal Distance on the right y-axis
yyaxis right
validRuns4_dist = ~isnan(maxHorizDistHist);
if any(validRuns4_dist)
    plot(pitchRates(validRuns4_dist), maxHorizDistHist(validRuns4_dist), 's-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'g', 'DisplayName', 'Max Horizontal Distance');
    ylabel('Maximum Horizontal Distance (km)');
else
    ylabel('Maximum Horizontal Distance (km) - No Data');
end

grid on;
xlabel('Initial Pitch Rate (°/s)');
legend('Location','best');
title('Max Altitude & Horizontal Distance vs Initial Pitch Rate');
saveas(gcf, fullfile(resultsDir, 'sweep_max_distances_vs_pitchrate.png'));

fprintf('Plotting complete. Figures saved to %s.\n', resultsDir);

% --- Display Timings --- (Already printed after each block)
if useParallel && exist('parsimTime', 'var')
    fprintf('\n--- Timing Summary ---\n');
    fprintf('Parallel (parsim) execution time: %.2f seconds\n', parsimTime);
elseif ~useParallel && exist('sequentialTime', 'var')
    fprintf('\n--- Timing Summary ---\n');
    fprintf('Sequential (sim) execution time: %.2f seconds\n', sequentialTime);
end

% --- Cleanup ---
fprintf('Closing model %s...\n', modelName);
close_system(modelName, 0); % Close model without saving changes
fprintf('Script finished.\n');
