% --- Script: Pitch Rate Sweep ---
clear; clc; close all; % Start fresh for a sweep script

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
% Load the model into memory once before the loop for efficiency (especially with Fast Restart)
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

% --- Simulation Loop --- Simplified, no waitbar ---
fprintf('\nStarting pitch rate sweep for %d runs...\n', totalRuns); % Use totalRuns

for i = 1:totalRuns % Loop using totalRuns
    % --- Store essential variables for this iteration ---
    currentRunIndex = i;
    currentPitchRate = pitchRates(i);
    % Generate legend entry for this run now
    currentLegendEntry = sprintf('PR=%.1f°/s', currentPitchRate);
    legendEntriesPlot1{currentRunIndex} = currentLegendEntry;
    legendEntriesPlot2{currentRunIndex} = currentLegendEntry;

    % --- Display Progress ---
    fprintf('Running simulation %d/%d with initial pitch rate = %.1f deg/s...\n', currentRunIndex, totalRuns, currentPitchRate);

    % --- Run Initialization INSIDE the loop ---
    % Use evalc to suppress command window output from these scripts
    try
        evalc('initialize_parameters'); 
        evalc('ControlSystemDesign');   
    catch ME_init
        fprintf('ERROR during initialization/design for run %d (PR=%.1f):\n', currentRunIndex, currentPitchRate);
        rethrow(ME_init);
    end

    % --- Verify and Update the specific parameter for this run ---
    % Check if 'Initial' exists and has 'Conditions' field after init scripts
    if ~exist('Initial','var') || ~isfield(Initial,'Conditions')
        error('Variable ''Initial.Conditions'' not found or ''Initial'' not defined after running initialize_parameters/ControlSystemDesign for run %d. Check those scripts.', currentRunIndex);
    end
    Initial.Conditions.pitchRate = currentPitchRate; % Override the specific value

    % --- Run Simulink model ---
    try
        simOut = sim(modelName, ...
                     'SrcWorkspace','current', ...
                     'ReturnWorkspaceOutputs','on');
    catch ME_sim
        fprintf('\n--- Simulation Error ---\n');
        fprintf('Simulation run %d failed for pitch rate %.1f deg/s.\n', currentRunIndex, currentPitchRate);
        fprintf('Error message: %s\n', ME_sim.message);
        for k=1:length(ME_sim.stack)
             fprintf('  File: %s, Name: %s, Line: %d\n', ME_sim.stack(k).file, ME_sim.stack(k).name, ME_sim.stack(k).line);
        end
        error('Simulation failed. Stopping script.');
    end

    % --- Extract and convert data to degrees ---
    try
        tRun           = simOut.tout;
        nRun           = simOut.NozzleAngle.Data * 180/pi;
        thRun          = simOut.theta.Data       * 180/pi;
        thCmdRun       = simOut.thetaCmd.Data    * 180/pi;
        xeRun          = simOut.Xe.Data; % Extract Earth-frame position data
    catch ME_extract
        fprintf('\n--- Data Extraction Error ---\n');
        fprintf('Failed to extract data after simulation run %d (PR=%.1f deg/s).\n', currentRunIndex, currentPitchRate);
        fprintf('Check the output signal names in the Simulink model (e.g., ''NozzleAngle'', ''theta'', ''thetaCmd'', ''Xe'').\n');
        fprintf('Error message: %s\n', ME_extract.message);
        error('Data extraction failed. Stopping script.');
    end

    % --- Calculate Max Horizontal Distance & Altitude for this run ---
    % Assumes Xe is Nx3 with columns [X, Y, Z_down]
    currentMaxHorizDist_m = max(sqrt(sum(xeRun(:,1:2).^2, 2))); % Calculate in meters
    currentMaxHorizDist_km = currentMaxHorizDist_m / 1000; % Convert to kilometers
    
    % If Z points down, min(Z) is the highest point. Negate for positive altitude.
    currentMaxAltitude_m  = -min(xeRun(:,3)); 
    currentMaxAltitude_km = currentMaxAltitude_m / 1000; % Convert to kilometers
    
    fprintf('    Run %d: Max Alt = %.3f km, Max Horiz Dist = %.3f km\n', currentRunIndex, currentMaxAltitude_km, currentMaxHorizDist_km); % DEBUG: Display calculated max values in km

    % --- Store results ---
    if currentRunIndex == 1
        timeVec = tRun;
    elseif length(tRun) ~= length(timeVec) || any(abs(tRun - timeVec) > 1e-9 * max(abs(tRun)))
        warning('Time vectors differ between simulation runs (Run %d). Interpolating results.', currentRunIndex);
        % Interpolate results to match the first time vector
        nRun = interp1(tRun, nRun, timeVec, 'linear', 'extrap');
        thRun = interp1(tRun, thRun, timeVec, 'linear', 'extrap');
        thCmdRun = interp1(tRun, thCmdRun, timeVec, 'linear', 'extrap');
        % We don't strictly need to interpolate Xe for max distance calculation,
        % but if other plots used interpolated Xe, it would be done here.
    end

    nozzleHist{currentRunIndex}   = nRun;
    thetaHist{currentRunIndex}    = thRun;
    thetaCmdHist{currentRunIndex} = thCmdRun;
    maxHorizDistHist(currentRunIndex) = currentMaxHorizDist_km; % Store max horizontal distance in KILOMETERS
    maxAltitudeHist(currentRunIndex)  = currentMaxAltitude_km;  % Store max altitude in KILOMETERS

    fprintf('Finished Run %d of %d.\n', currentRunIndex, totalRuns);

end

fprintf('...Pitch rate sweep complete.\n');

% --- Plotting Results --- Use totalRuns and stored legend entries ---
fprintf('Generating plots...\n');

% Plot 1: Nozzle angle vs time for all runs
figure('Name','Nozzle Angle Time Histories');
hold on;
colors1 = lines(totalRuns);
for i = 1:totalRuns
    plot(timeVec, nozzleHist{i}, 'LineWidth', 1.5, 'Color', colors1(i,:));
end
hold off;
grid on;
xlabel('Time (s)');
ylabel('Nozzle Deflection (°)');
legend(legendEntriesPlot1, 'Location','best'); % Use stored legend entries
title('Nozzle Angle vs Time for Various Initial Pitch Rates');
saveas(gcf, fullfile(resultsDir, 'sweep_nozzle_histories.png'));

% Plot 2: θ and θ_cmd vs time for all runs
figure('Name','Pitch Angle vs Command Time Histories');
hold on;
colors2 = lines(totalRuns);
legendHandles2 = gobjects(totalRuns, 1);
for i = 1:totalRuns
    h_theta = plot(timeVec, thetaHist{i},   '-',  'Color', colors2(i,:), 'LineWidth', 1.5);
    plot(timeVec, thetaCmdHist{i}, '--', 'Color', colors2(i,:), 'LineWidth', 1.0);
    legendHandles2(i) = h_theta;
end
hold off;
grid on;
xlabel('Time (s)');
ylabel('Angle (°)');
legend(legendHandles2, legendEntriesPlot2, 'Location','best'); % Use stored legend entries
title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', 'for Various Initial Pitch Rates'});
saveas(gcf, fullfile(resultsDir, 'sweep_pitch_histories.png'));

% Plot 3: Max nozzle deflection vs initial pitch-rate
% Calculate max absolute deflection for each run
maxNozzleAbs = cellfun(@(x) max(abs(x)), nozzleHist);

figure('Name','Max Nozzle Angle vs Initial Pitch Rate');
% Check if pitchRates still exists for the x-axis. If not, this plot will fail.
% If init scripts clear pitchRates, it needs to be re-defined or saved/loaded.
if ~exist('pitchRates', 'var')
    warning('pitchRates variable was cleared. Reloading from original definition for plot 3.');
    pitchRates = 0:0.5:5; % Re-define if cleared
end
plot(pitchRates, maxNozzleAbs, 'o-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'b');
grid on;
xlabel('Initial Pitch Rate (°/s)');
ylabel('Maximum Absolute Nozzle Deflection (°)');
title('Max |Nozzle Deflection| vs Initial Pitch Rate');
saveas(gcf, fullfile(resultsDir, 'sweep_max_nozzle_vs_pitchrate.png'));

% Plot 4: Max Altitude & Horizontal Distance vs Initial Pitch Rate
figure('Name','Max Altitude & Horizontal Distance vs Initial Pitch Rate');

if ~exist('pitchRates', 'var')
    warning('pitchRates variable was cleared. Reloading from original definition for plot 4.');
    pitchRates = 0:0.25:1; % Re-define if cleared (adjust step if needed)
end

% Plot Max Altitude on the left y-axis
yyaxis left
plot(pitchRates, maxAltitudeHist, 'o-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'r', 'DisplayName', 'Max Altitude');
ylabel('Maximum Altitude (km)'); % Update Y-axis label to km

% Plot Max Horizontal Distance on the right y-axis
yyaxis right
plot(pitchRates, maxHorizDistHist, 's-', 'LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor', 'g', 'DisplayName', 'Max Horizontal Distance');
ylabel('Maximum Horizontal Distance (km)'); % Update Y-axis label to km

% General plot settings
grid on;
xlabel('Initial Pitch Rate (°/s)');
legend('Location','best');
title('Max Altitude & Horizontal Distance vs Initial Pitch Rate');
saveas(gcf, fullfile(resultsDir, 'sweep_max_distances_vs_pitchrate.png')); % Keep same filename or change if preferred

fprintf('Plotting complete. Figures saved to %s.\n', resultsDir);
