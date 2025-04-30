% --- Script: Plot Pitch Rate Sweep Results ---
clear; clc; close all; % Start fresh

% --- Configuration ---
savePlots = false; % Set to true to save plots, false to only display them

% --- Setup: Robust Path Configuration ---
fprintf('Setting up paths...\n');
scriptPath = mfilename('fullpath');
[scriptDir, ~, ~] = fileparts(scriptPath); % Gets '/.../STEVE_Simulator/src/scripts'

% Navigate up to the project root directory (assuming scripts is in src)
projectRoot = fullfile(scriptDir, '..', '..'); % Should be '/.../STEVE_Simulator'

fprintf('Script directory: %s\n', scriptDir);
fprintf('Project root identified as: %s\n', projectRoot);

% Define key directories relative to the project root
resultsDir = fullfile(projectRoot, 'data', 'results');
fprintf('Results directory: %s\n', resultsDir);

% --- Load Latest Results File ---
resultsPattern = fullfile(resultsDir, 'pitch_rate_sweep_results_*.mat');
resultsFiles = dir(resultsPattern);

if isempty(resultsFiles)
    error('No pitch rate sweep result files found in %s matching pattern %s', resultsDir, resultsPattern);
end

% Sort by date to find the latest file
[~, latestIdx] = max([resultsFiles.datenum]);
latestResultsFile = fullfile(resultsDir, resultsFiles(latestIdx).name);

fprintf('Loading latest results from: %s\n', latestResultsFile);
try
    load(latestResultsFile);
    fprintf('Results loaded successfully.\n');
catch ME_load
    fprintf('Error loading results file: %s\n', ME_load.message);
    rethrow(ME_load);
end

% --- Verify Loaded Variables ---
requiredVars = {'pitchRates', 'totalRuns', 'timeVec', 'nozzleHist', 'thetaHist', ...
                'thetaCmdHist', 'maxHorizDistHist', 'maxAltitudeHist'};
missingVars = setdiff(requiredVars, who); % Check which required variables are NOT in the workspace
if ~isempty(missingVars)
    error('The loaded results file %s is missing the following required variables: %s', ...
          latestResultsFile, strjoin(missingVars, ', '));
end
fprintf('All required variables found in the loaded data.\n');

% --- Generate Legend Entries ---
fprintf('Generating legend entries...\n');
legendEntriesPlot1 = cell(totalRuns, 1);
legendEntriesPlot2 = cell(totalRuns, 1);
for i = 1:totalRuns
     % Check if pitchRates exists and has enough elements
    if ~exist('pitchRates', 'var') || i > numel(pitchRates)
        warning('pitchRates variable issue for run %d. Using default legend.', i);
        currentLegendEntry = sprintf('Run %d', i);
    else
        currentLegendEntry = sprintf('PR=%.1f°/s', pitchRates(i));
    end
    legendEntriesPlot1{i} = currentLegendEntry;
    legendEntriesPlot2{i} = currentLegendEntry;
end
fprintf('Legend entries generated.\n');


% --- Plotting Results --- (Uses the loaded and verified data)
fprintf('Generating plots...\n');

% Plot 1: Nozzle angle vs time for all runs
figure('Name','Nozzle Angle Time Histories');
hold on;
colors1 = lines(totalRuns);
% Find runs with valid (non-NaN) data in nozzleHist
validRunIndices1 = find(cellfun(@(x) ~isempty(x) && ~all(isnan(x(:))), nozzleHist));
if isempty(validRunIndices1)
    title('Nozzle Angle vs Time (No valid runs found in data)');
    warning('No valid data found in nozzleHist for Plot 1.');
else
    legendHandles1 = gobjects(length(validRunIndices1), 1);
    legendTexts1 = cell(length(validRunIndices1), 1);
    plotIdx = 1;
    for i = validRunIndices1(:)' % Iterate through valid runs
        if i <= numel(nozzleHist) && ~isempty(nozzleHist{i}) && i <= numel(legendEntriesPlot1)
            legendHandles1(plotIdx) = plot(timeVec, nozzleHist{i}, 'LineWidth', 1.5, 'Color', colors1(i,:));
            legendTexts1{plotIdx} = legendEntriesPlot1{i};
            plotIdx = plotIdx + 1;
        else
             warning('Skipping plot for run index %d in Plot 1 due to inconsistent data.', i);
        end
    end
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Nozzle Deflection (°)');
    if plotIdx > 1 % Check if any lines were actually plotted
        legend(legendHandles1(1:plotIdx-1), legendTexts1(1:plotIdx-1), 'Location','best');
        title('Nozzle Angle vs Time for Various Initial Pitch Rates');
    else
        title('Nozzle Angle vs Time (No valid runs plotted)');
    end
end
if savePlots
    saveas(gcf, fullfile(resultsDir, 'sweep_nozzle_histories.png'));
    fprintf('Plot 1 saved.\n');
else
    fprintf('Plot 1 generated (not saved).\n');
end

% Plot 2: Box plot of Nozzle Angle Distribution vs Initial Pitch Rate (with Max Trend)
figure('Name', 'Nozzle Angle Distribution vs Initial Pitch Rate');

% Prepare data for boxplot
allNozzleData = [];
pitchRateGroup = [];
validRunIndicesBox = find(cellfun(@(x) ~isempty(x) && ~all(isnan(x(:))), nozzleHist));
validPitchRates = [];
maxNozzlePerRate = containers.Map('KeyType','double','ValueType','double');

for i = validRunIndicesBox(:)'
    if i <= numel(nozzleHist) && i <= numel(pitchRates)
        currentNozzleData = nozzleHist{i}(:); % Ensure column vector
        currentPitchRate = pitchRates(i);
        allNozzleData = [allNozzleData; currentNozzleData];
        pitchRateGroup = [pitchRateGroup; repmat(currentPitchRate, numel(currentNozzleData), 1)];
        
        currentMaxAbs = max(abs(currentNozzleData));
        if ~isKey(maxNozzlePerRate, currentPitchRate) || currentMaxAbs > maxNozzlePerRate(currentPitchRate)
             maxNozzlePerRate(currentPitchRate) = currentMaxAbs;
        end
        
        if ~ismember(currentPitchRate, validPitchRates)
             validPitchRates = [validPitchRates, currentPitchRate]; % Keep track of pitch rates actually plotted
        end
    end
end

% Get keys (cell array of doubles), convert to numeric array, then sort
rateKeysCell = keys(maxNozzlePerRate);
if isempty(rateKeysCell)
    sortedUniqueRates = [];
    maxValsToPlot = [];
else
    numericRateKeys = cell2mat(rateKeysCell);
    sortedUniqueRates = sort(numericRateKeys); % Now sorting a numeric array
    % Get values corresponding to the sorted keys
    maxValsToPlot = cell2mat(values(maxNozzlePerRate, num2cell(sortedUniqueRates))); 
end

if ~isempty(allNozzleData)
    boxplot(allNozzleData, pitchRateGroup, 'Labels', arrayfun(@(x) sprintf('%.1f', x), sort(unique(pitchRateGroup)), 'UniformOutput', false));
    grid on;
    xlabel('Initial Pitch Rate (°/s)');
    ylabel('Nozzle Deflection (°)');
    title('Distribution of Nozzle Angles vs Initial Pitch Rate');
    ylimVal = max(abs(ylim)); % Get current max y-limit after boxplot
    ylim([-ylimVal, ylimVal]); % Center y-axis around 0
else
    text(0.5, 0.5, 'No valid runs to plot', 'HorizontalAlignment', 'center');
    title('Distribution of Nozzle Angles vs Initial Pitch Rate (No valid data)');
    warning('No valid data found for Plot 2 (Box Plot).');
end

if savePlots
    saveas(gcf, fullfile(resultsDir, 'sweep_nozzle_boxplot_vs_pitchrate.png'));
    fprintf('Plot 2 saved.\n');
else
    fprintf('Plot 2 generated (not saved).\n');
end

% Plot 3: θ and θ_cmd vs time for all runs (Previously Plot 2)
figure('Name','Pitch Angle vs Command Time Histories');
hold on;
colors3 = lines(totalRuns); % Use new color index if desired, or reuse colors2
% Find runs with valid (non-NaN) data in thetaHist
validRunIndices3 = find(cellfun(@(x) ~isempty(x) && ~all(isnan(x(:))), thetaHist));
if isempty(validRunIndices3)
    title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', '(No valid runs found in data)'});
    warning('No valid data found in thetaHist for Plot 3.');
else
    legendHandles3 = gobjects(length(validRunIndices3), 1);
    legendTexts3 = cell(length(validRunIndices3), 1);
    plotIdx = 1;
    for i = validRunIndices3(:)' % Iterate through valid runs
         if i <= numel(thetaHist) && ~isempty(thetaHist{i}) && i <= numel(legendEntriesPlot2) % Still use legendEntriesPlot2 as it corresponds to runs
            h_theta = plot(timeVec, thetaHist{i}, '-', 'Color', colors3(i,:), 'LineWidth', 1.5);
            % Check if thetaCmdHist{i} is also valid before plotting
            if i <= length(thetaCmdHist) && ~isempty(thetaCmdHist{i}) && ~all(isnan(thetaCmdHist{i}(:)))
                plot(timeVec, thetaCmdHist{i}, '--', 'Color', colors3(i,:), 'LineWidth', 1.0);
            end
            legendHandles3(plotIdx) = h_theta; % Use handle from actual theta plot for legend
            legendTexts3{plotIdx} = legendEntriesPlot2{i}; % Still use legendEntriesPlot2
            plotIdx = plotIdx + 1;
         else
             warning('Skipping plot for run index %d in Plot 3 due to inconsistent data.', i);
         end
    end
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Angle (°)');
    if plotIdx > 1 % Check if any lines were actually plotted
        legend(legendHandles3(1:plotIdx-1), legendTexts3(1:plotIdx-1), 'Location','best');
        title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', 'for Various Initial Pitch Rates'});
    else
        title({'Actual (Solid) and Commanded (Dashed) Pitch Angles', '(No valid runs plotted)'});
    end
end
if savePlots
    saveas(gcf, fullfile(resultsDir, 'sweep_pitch_histories.png'));
    fprintf('Plot 3 saved.\n');
else
    fprintf('Plot 3 generated (not saved).\n');
end

% Plot 4: Max Altitude, Horizontal Distance, and Nozzle Deflection vs Initial Pitch Rate (Manual 3-Axis)
figure('Name','Max Performance Metrics vs Initial Pitch Rate');

if ~exist('pitchRates', 'var')
    error('pitchRates variable not loaded from results file for Plot 4.');
end
if ~exist('maxAltitudeHist', 'var') || ~exist('maxHorizDistHist', 'var')
     error('maxAltitudeHist or maxHorizDistHist not loaded from results file for Plot 4.');
end

% Calculate max absolute nozzle deflection, handling potential empty/NaN cells
maxNozzleAbs = zeros(totalRuns, 1) * NaN; % Initialize with NaN
for i = 1:totalRuns
    if i <= numel(nozzleHist) && ~isempty(nozzleHist{i}) && ~all(isnan(nozzleHist{i}(:)))
        maxNozzleAbs(i) = max(abs(nozzleHist{i}));
    else
        % Ensure it remains NaN if data is missing or invalid for this run
        maxNozzleAbs(i) = NaN;
    end
end

% Define colors
colorAlt = [1 0 0]; % Red
colorDist = [0 1 0]; % Green
colorNozzle = [0 0 1]; % Blue

legendHandles = [];
legendLabels = {};

% --- Axis 1 (Altitude) ---
ax1 = gca; % Get current axes
hold(ax1, 'on');
validRunsAlt = ~isnan(maxAltitudeHist);
if any(validRunsAlt)
    h1 = plot(ax1, pitchRates(validRunsAlt), maxAltitudeHist(validRunsAlt), 'o-', ...
              'Color', colorAlt, 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colorAlt);
    legendHandles(end+1) = h1;
    legendLabels{end+1} = 'Max Altitude (km)';
    ylabel(ax1, 'Maximum Altitude (km)');
else
    ylabel(ax1, 'Maximum Altitude (km) - No Data');
    warning('No valid altitude data found for Plot 4.');
end
set(ax1, 'XColor', 'k', 'YColor', colorAlt, 'Box', 'off'); % Color axis 1
grid(ax1, 'on');
xlabel(ax1, 'Initial Pitch Rate (°/s)'); % Label X on the first axis

% --- Axis 2 (Distance) ---
ax2 = axes('Position', get(ax1, 'Position'), ...
           'YAxisLocation', 'right', ...
           'Color', 'none', ... % Transparent background
           'XAxisLocation', 'bottom', ... % Place X axis at bottom (like ax1)
           'XTick', [], ... % Hide X ticks and labels for this axis
           'YColor', colorDist, ...
           'Box', 'off');
linkaxes([ax1, ax2], 'x'); % Link X axes
hold(ax2, 'on');
validRunsDist = ~isnan(maxHorizDistHist);
if any(validRunsDist)
    h2 = plot(ax2, pitchRates(validRunsDist), maxHorizDistHist(validRunsDist), 's-', ...
              'Color', colorDist, 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colorDist);
    legendHandles(end+1) = h2;
    legendLabels{end+1} = 'Max Horizontal Distance (km)';
    ylabel(ax2, 'Maximum Horizontal Distance (km)');
else
     ylabel(ax2, 'Maximum Horizontal Distance (km) - No Data');
     warning('No valid horizontal distance data found for Plot 4.');
end
hold(ax2, 'off');

% --- Axis 3 (Nozzle Angle) ---
pos1 = get(ax1, 'Position'); % Get position of the base axis
ax3 = axes('Position', pos1, ... % Start with same position as ax1
           'YAxisLocation', 'right', ...
           'Color', 'none', ...
           'XAxisLocation', 'bottom', ... % Keep X axis at bottom (invisible)
           'XTick', [], ...
           'YColor', colorNozzle, ...
           'Box', 'off');

% Link all three x-axes together
linkaxes([ax1, ax2, ax3], 'x'); 

% Offset the third axis slightly to the right to separate labels
offset_factor = 0.08; % Adjust this value if needed (0.05 to 0.1 is typical)
pos3 = get(ax3, 'Position');
set(ax3, 'Position', [pos3(1) + offset_factor, pos3(2), pos3(3) - offset_factor, pos3(4)]); % Shift right and slightly reduce width

hold(ax3, 'on');
validRunsNozzle = ~isnan(maxNozzleAbs);
if any(validRunsNozzle)
    h3 = plot(ax3, pitchRates(validRunsNozzle), maxNozzleAbs(validRunsNozzle), 'd-', ...
              'Color', colorNozzle, 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', colorNozzle);
    legendHandles(end+1) = h3;
    legendLabels{end+1} = 'Max |Nozzle Angle| (°)';
    ylabel(ax3, 'Max |Nozzle Angle| (°)');
else
    ylabel(ax3, 'Max |Nozzle Angle| (°) - No Data');
    warning('No valid max nozzle angle data found for Plot 4.');
end
hold(ax3, 'off');

% --- Stacking Order --- 
% Bring ax2 above ax3 (so its line/markers aren't hidden by ax3's axes)
% Bring ax1 to the very top (for grid lines)
uistack(ax2, 'top'); 
uistack(ax1, 'top');

hold(ax1, 'off'); % Release hold on the primary axis
title(ax1, 'Max Performance Metrics vs Initial Pitch Rate');

% Add legend using collected handles and labels
if ~isempty(legendHandles)
    legend(legendHandles, legendLabels, 'Location', 'best');
end

% --- Save Plot ---
if savePlots
    saveas(gcf, fullfile(resultsDir, 'sweep_max_metrics_vs_pitchrate.png'));
    fprintf('Plot 4 saved.\n');
else
    fprintf('Plot 4 generated (not saved).\n');
end

fprintf('\nPlotting script finished.\n');
