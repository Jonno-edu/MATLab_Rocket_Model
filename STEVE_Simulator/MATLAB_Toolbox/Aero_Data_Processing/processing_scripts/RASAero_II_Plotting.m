% --- Script to Plot and Interpolate RASAero II Data ---
clear; clc; close all;

% --- Configuration ---
excelFilePath = 'UTF-8RASAeroII25.xlsx'; % Ensure this file is in MATLAB's path or provide the full path
sheetName = 'Sheet1';

% Coefficients to process and plot (using actual MATLAB variable names)
coeffNames = {'CN', 'CAPower_On', 'CP', 'CNalpha_0To4Deg__perRad_'};

% Create interpreter-safe names for plot labels/titles
coeffNamesSafe = strrep(coeffNames, '_', '\_');
coeffNamesSafe = strrep(coeffNamesSafe, '\_\_', '\_'); % Handle double underscore

% Constants
inch_to_meter = 0.0254;

% Interpolation settings
interpolationMethod = 'natural';
extrapolationMethod = 'nearest';

fprintf('Loading data from "%s", Sheet: "%s"\n', excelFilePath, sheetName);

% --- Load Data ---
try
    opts = detectImportOptions(excelFilePath, 'Sheet', sheetName);
    rasAeroData = readtable(excelFilePath, opts);
    fprintf('Data loaded successfully. Found %d rows and %d columns.\n', height(rasAeroData), width(rasAeroData));

    % --- Verify Required Columns Exist ---
    requiredCols = [{'Mach', 'Alpha'}, coeffNames];
    missingCols = setdiff(requiredCols, rasAeroData.Properties.VariableNames);
    if ~isempty(missingCols)
        error('Missing required columns in the loaded data: %s', strjoin(missingCols, ', '));
    end

    % --- Convert CP to Meters ---
    if ismember('CP', rasAeroData.Properties.VariableNames)
        fprintf('Converting CP column from inches to meters...\n');
        rasAeroData.CP = rasAeroData.CP * inch_to_meter; % Overwrite original CP column
        % Update the safe name for CP if needed for z-label
        cpIndex = find(strcmp(coeffNames, 'CP'));
        if ~isempty(cpIndex)
            coeffNamesSafe{cpIndex} = 'CP (m)';
        end
    else
        warning('CP column not found. Cannot convert to meters.');
        % Remove CP from list if conversion is critical and column is missing
        cpIndex = find(strcmp(coeffNames, 'CP'));
        if ~isempty(cpIndex)
            coeffNames(cpIndex) = [];
            coeffNamesSafe(cpIndex) = [];
            warning('Removed CP from processing list as it was not found.');
        end
    end

catch ME
    error('Failed to load or process initial data from Excel file "%s". Error: %s', excelFilePath, ME.message);
end


% --- Define Target Grid Based on Loaded Data ---
fprintf('Defining target interpolation grid...\n');
numMachPoints = 50; % Adjust grid density as needed
numAlphaPoints = 40;

% Get min/max from the loaded RASAero data
minGridMach = min(rasAeroData.Mach);
maxGridMach = max(rasAeroData.Mach);
minGridAlpha = min(rasAeroData.Alpha);
maxGridAlpha = max(rasAeroData.Alpha);

targetMach = linspace(minGridMach, maxGridMach, numMachPoints);
targetAlpha = linspace(minGridAlpha, maxGridAlpha, numAlphaPoints);

[MachGrid, AlphaGrid] = meshgrid(targetMach, targetAlpha);
fprintf('Target grid defined (Mach: %d points, Alpha: %d points).\n', numMachPoints, numAlphaPoints);


% --- Interpolate onto Target Grid ---
fprintf('Interpolating coefficients onto grid using "%s" method...\n', interpolationMethod);
interpolatedTables = struct();

% Extract source data vectors once
sourceMach = rasAeroData.Mach;
sourceAlpha = rasAeroData.Alpha;

for i = 1:length(coeffNames) % Loop through the potentially updated coeffNames list
    coeff = coeffNames{i};
    fprintf('  Interpolating %s...\n', coeff);
    sourceCoeffData = rasAeroData.(coeff); % Get the (potentially converted) coefficient data

    try
        F_interp = scatteredInterpolant(sourceMach, sourceAlpha, sourceCoeffData, ...
                                        interpolationMethod, extrapolationMethod);
        interpolatedTables.(coeff) = F_interp(MachGrid, AlphaGrid);
         fprintf('  Done.\n');
    catch ME
        warning('Interpolation failed for %s with method %s: %s. Skipping coefficient.', coeff, interpolationMethod, ME.message);
         if isfield(interpolatedTables, coeff)
            interpolatedTables = rmfield(interpolatedTables, coeff);
        end
    end
end
fprintf('Interpolation complete.\n');

% Check which coefficients were successfully interpolated
finalCoeffNames = fieldnames(interpolatedTables);
if isempty(finalCoeffNames)
    warning('No coefficients were successfully interpolated. No plots will be generated.');
else
    finalCoeffNamesSafe = strrep(finalCoeffNames, '_', '\_');
    finalCoeffNamesSafe = strrep(finalCoeffNamesSafe, '\_\_', '\_'); % Handle double underscore
    % Update CP label in safe names if present
    cpIndex = find(strcmp(finalCoeffNames, 'CP'));
     if ~isempty(cpIndex)
         finalCoeffNamesSafe{cpIndex} = 'CP (m)';
     end

    % --- Combined Visualization ---
    fprintf('Generating combined scatter plots (Original RASAero + Interpolated Grid)...\n');

    % Original data plot settings
    colorOrig = 'blue';
    markerSizeOrig = 20;

    % Interpolated grid plot settings
    markerGrid = 'o';
    markerSizeGrid = 30;
    markerAlphaGrid = 0.7;

    % Loop through each successfully interpolated coefficient
    for i = 1:length(finalCoeffNames)
        coeff = finalCoeffNames{i};
        coeffSafe = finalCoeffNamesSafe{i};

        figure;
        hold on;

        % --- Plot 1: Interpolated Grid Points (Gradient Color) ---
        coeffDataGrid = interpolatedTables.(coeff);
        mach_vector_grid = MachGrid(:);
        alpha_vector_grid = AlphaGrid(:);
        coeff_vector_grid = coeffDataGrid(:);
        zLabelString = coeffSafe; % Use the safe name which might include (m) for CP

        scatter3(mach_vector_grid, alpha_vector_grid, coeff_vector_grid, ...
                 markerSizeGrid, ...
                 coeff_vector_grid, ... % Color by value
                 markerGrid, ...
                 'filled', ...
                 'MarkerFaceAlpha', markerAlphaGrid, ...
                 'DisplayName', ['Interpolated Grid (', interpolationMethod, ')']);

        % --- Plot 2: Original RASAero Data Points (Single Color) ---
        sourceCoeffDataPlot = rasAeroData.(coeff); % Get original data again for plotting
        scatter3(sourceMach, sourceAlpha, sourceCoeffDataPlot, ...
                 markerSizeOrig, colorOrig, 'filled', ...
                 'DisplayName', 'Original RASAero Data');

        % --- Add Labels, Title, Colorbar, etc. ---
        xlabel('Mach');
        ylabel('Alpha (deg)');
        zlabel(zLabelString, 'Interpreter', 'tex');
        title(['RASAero & Interpolated Grid (Method: ', interpolationMethod, '): ', coeffSafe, ' vs Mach vs Alpha'], 'Interpreter', 'tex');
        colormap(gca, 'parula');
        colorbar;
        legend('show', 'Location', 'best');
        grid on;
        view(-30, 30);
        hold off;

        fprintf('  Generated combined plot for %s.\n', coeff);
    end
    fprintf('All combined plots generated.\n');
end % End check for successfully interpolated coefficients

% --- Optional: Save Interpolated Data (Commented Out) ---
% If you want to save the interpolated RASAero data for later use:
% fprintf('Packaging and saving interpolated RASAero data...\n');
% RASAeroInterpolatedData = struct();
% RASAeroInterpolatedData.Breakpoints.Mach = targetMach(:)';
% RASAeroInterpolatedData.Breakpoints.Alpha = targetAlpha(:)';
% RASAeroInterpolatedData.Tables = interpolatedTables; % Contains only successfully interpolated tables
% RASAeroInterpolatedData.Info.SourceFile = excelFilePath;
% RASAeroInterpolatedData.Info.InterpolationMethod = interpolationMethod;
% RASAeroInterpolatedData.Info.ExtrapolationMethod = extrapolationMethod;
% RASAeroInterpolatedData.Info.GridPoints = [numMachPoints, numAlphaPoints];
%
% outputMatFileName = 'RASAeroInterpolated_Grid.mat';
% save(outputMatFileName, 'RASAeroInterpolatedData');
% fprintf('Saved interpolated RASAero data to %s\n', outputMatFileName);


fprintf('\n--- Processing Finished ---\n');
