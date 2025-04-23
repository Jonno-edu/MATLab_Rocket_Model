% --- Script to Combine, Interpolate (Natural Method), Plot (Gradient), and Export ---
clear; clc; close all;

fprintf('Loading raw aerodynamic data...\n');
% --- Load Raw Data ---
excelFilePath = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data/STeVe V1 No Fins.xlsx';

% Import options for "Aero Properties" (0-4 deg)
optsS2 = spreadsheetImportOptions("NumVariables", 15);
optsS2.Sheet = "Aero Properties";
optsS2.DataRange = "A2:O7501";
optsS2.VariableNames = ["Mach", "Alpha", "CD", "CDPower_Off", "CDPower_ON", "CAPower_Off", "CAPower_On", "CL", "CN", "CNPotential", "CNViscous", "Cnalpha_0_4deg__perRad_", "CP", "CP_0_4deg_", "Reynolds"];
optsS2.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];
optsS2 = setvaropts(optsS2, "Reynolds", "WhitespaceRule", "preserve");
optsS2 = setvaropts(optsS2, "Reynolds", "EmptyFieldRule", "auto");
STeVeV1NoFinsS2 = readtable(excelFilePath, optsS2, "UseExcel", false);

% Import options for "Aero Properties 15deg" (0-15 deg)
optsS3 = spreadsheetImportOptions("NumVariables", 15);
optsS3.Sheet = "Aero Properties 15deg";
optsS3.DataRange = "A2:O81";
optsS3.VariableNames = optsS2.VariableNames;
optsS3.VariableTypes = optsS2.VariableTypes;
optsS3.VariableTypes{15} = 'string'; % Match S2 type
optsS3 = setvaropts(optsS3, "Reynolds", "WhitespaceRule", "preserve");
optsS3 = setvaropts(optsS3, "Reynolds", "EmptyFieldRule", "auto");
STeVeV1NoFinsS3 = readtable(excelFilePath, optsS3, "UseExcel", false);
fprintf('Raw data loaded.\n');

% --- Define Constants and Parameters ---
% Define the coefficients required (REMOVED CAPower_Off)
coeffNames = {'CN', 'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'}; % Removed 'CAPower_Off'
fprintf('Processing coefficients: %s\n', strjoin(coeffNames, ', '));

% Create interpreter-safe names for plotting labels/titles
coeffNamesSafe = strrep(coeffNames, '_', '\_');

% Define Mach limit
machLimit = 7.0;
fprintf('Applying Mach limit: Mach <= %.1f\n', machLimit);

% Conversion factor for CP
inch_to_meter = 0.0254;

% Define the chosen interpolation method
interpolationMethod = 'natural'; % Using 'natural' neighbor interpolation
extrapolationMethod = 'nearest'; % Keep extrapolation consistent

fprintf('Chosen Interpolation Method: %s\n', interpolationMethod);

% --- Filter Raw Data (Apply Mach Limit Early) ---
S2_filtered = STeVeV1NoFinsS2(STeVeV1NoFinsS2.Mach <= machLimit, :);
S3_filtered = STeVeV1NoFinsS3(STeVeV1NoFinsS3.Mach <= machLimit, :);
fprintf('Raw data filtered by Mach limit.\n');

% --- Prepare Data for Combination ---
fprintf('Preparing and combining filtered data for interpolation...\n');
S2_for_combine = S2_filtered(S2_filtered.Alpha <= 4, :);
S3_for_combine = S3_filtered(S3_filtered.Alpha > 4, :);
combinedMach = [S2_for_combine.Mach; S3_for_combine.Mach];
combinedAlpha = [S2_for_combine.Alpha; S3_for_combine.Alpha];
combinedCoeffs = struct();
% Loop uses the updated coeffNames list (without CAPower_Off)
for i = 1:length(coeffNames)
    coeff = coeffNames{i};
    if ismember(coeff, S2_for_combine.Properties.VariableNames) && ismember(coeff, S3_for_combine.Properties.VariableNames)
        tempCoeffData = [S2_for_combine.(coeff); S3_for_combine.(coeff)];
        if strcmp(coeff, 'CP')
            fprintf('  Converting combined CP from inches to meters for processing...\n');
            tempCoeffData = tempCoeffData * inch_to_meter;
        end
        combinedCoeffs.(coeff) = tempCoeffData;
    else
        warning('Coefficient %s not found consistently for processing, skipping.', coeff);
    end
end
if isempty(combinedMach) || isempty(combinedAlpha)
    error('No data points remain after filtering for combination. Check Mach/Alpha ranges.');
end
[uniqueMA, ia, ~] = unique([combinedMach, combinedAlpha], 'rows', 'stable');
combinedMach = uniqueMA(:, 1);
combinedAlpha = uniqueMA(:, 2);
activeCoeffNames = fieldnames(combinedCoeffs);
for i = 1:length(activeCoeffNames)
     coeff = activeCoeffNames{i};
     combinedCoeffs.(coeff) = combinedCoeffs.(coeff)(ia);
end
fprintf('Data preparation complete. %d unique data points for interpolation.\n', length(combinedMach));

% --- Define Target Grid ---
fprintf('Defining target interpolation grid...\n');
numMachPoints = 60;
numAlphaPoints = 50;
minGridMach = min(combinedMach);
maxGridMach = max(combinedMach);
targetMach = linspace(minGridMach, maxGridMach, numMachPoints);
minGridAlpha = min(combinedAlpha);
maxGridAlpha = max(combinedAlpha);
if minGridAlpha > 0
    warning('Minimum combined Alpha is %.2f > 0. Grid starts from here.', minGridAlpha);
end
if maxGridAlpha < 15
     warning('Maximum combined Alpha is %.2f < 15. Grid ends here.', maxGridAlpha);
end
targetAlpha = linspace(minGridAlpha, maxGridAlpha, numAlphaPoints);
[MachGrid, AlphaGrid] = meshgrid(targetMach, targetAlpha);
fprintf('Target grid defined (Mach: %d points, Alpha: %d points).\n', numMachPoints, numAlphaPoints);

% --- Interpolate onto Target Grid (Using chosen 'natural' method) ---
fprintf('Interpolating coefficients onto grid using "%s" method...\n', interpolationMethod);
interpolatedTables = struct();

currentActiveCoeffNames = fieldnames(combinedCoeffs); % Will not include CAPower_Off
for i = 1:length(currentActiveCoeffNames)
    coeff = currentActiveCoeffNames{i};
    fprintf('  Interpolating %s...\n', coeff);
    try
        F_interp = scatteredInterpolant(combinedMach, combinedAlpha, combinedCoeffs.(coeff), ...
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

finalCoeffNames = fieldnames(interpolatedTables);
if isempty(finalCoeffNames)
    error('No coefficients were successfully interpolated with method "%s". Halting execution.', interpolationMethod);
end

% --- Package Data for Export ---
fprintf('Packaging data for export...\n');
CombinedAeroData = struct();
CombinedAeroData.Breakpoints = struct();
CombinedAeroData.Tables = struct();
CombinedAeroData.Breakpoints.Mach = targetMach(:)';
CombinedAeroData.Breakpoints.Alpha = targetAlpha(:)';
% Loop uses finalCoeffNames, which won't include CAPower_Off
for i = 1:length(finalCoeffNames)
     coeff = finalCoeffNames{i};
     CombinedAeroData.Tables.(coeff) = interpolatedTables.(coeff);
end
CombinedAeroData.Info.Description = sprintf('Combined and interpolated aero data (Alpha %.2f-%.2f deg, Mach %.2f-%.2f)', minGridAlpha, maxGridAlpha, minGridMach, maxGridMach);
CombinedAeroData.Info.SourceFiles = {optsS2.Sheet, optsS3.Sheet};
CombinedAeroData.Info.GenerationDate = datetime('now');
CombinedAeroData.Info.InterpolationMethod = interpolationMethod;
CombinedAeroData.Info.ExtrapolationMethod = extrapolationMethod;
fprintf('Data packaged.\n');

% --- Export Data to MAT File ---
targetFolder = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data';
outputFileName = fullfile(targetFolder, 'CombinedAeroData_Grid.mat');
fprintf('Saving combined data (using %s interpolation) to %s...\n', interpolationMethod, outputFileName);
save(outputFileName, 'CombinedAeroData'); % CAPower_Off will not be in this structure
fprintf('Data saved successfully.\n');

% --- Combined Visualization: Raw (Color-Coded) and Interpolated Grid Points (Gradient Color) ---
fprintf('Generating combined scatter plots (Raw data + Interpolated Grid using %s with gradient color)...\n', interpolationMethod);
finalCoeffNamesSafe = strrep(finalCoeffNames, '_', '\_'); % Will not include CAPower_Off

% Raw data plot settings
colorS2 = 'blue';
colorS3 = 'red';
markerSizeRaw = 20;

% Interpolated grid plot settings
markerGrid = 'o';
markerSizeGrid = 30;
markerAlphaGrid = 0.7;

% Loop uses finalCoeffNames, which won't include CAPower_Off
for i = 1:length(finalCoeffNames)
    coeff = finalCoeffNames{i};
    coeffSafe = finalCoeffNamesSafe{i};

    figure;
    hold on;

    % --- Plot 1: Interpolated Grid Points (Gradient Color) ---
    coeffDataGrid = CombinedAeroData.Tables.(coeff);
    mach_vector_grid = MachGrid(:);
    alpha_vector_grid = AlphaGrid(:);
    coeff_vector_grid = coeffDataGrid(:);
    zLabelString = coeffSafe;
    if strcmp(coeff, 'CP')
        zLabelString = 'CP (m)';
    end

    scatter3(mach_vector_grid, alpha_vector_grid, coeff_vector_grid, ...
             markerSizeGrid, ...
             coeff_vector_grid, ...
             markerGrid, ...
             'filled', ...
             'MarkerFaceAlpha', markerAlphaGrid, ...
             'DisplayName', ['Interpolated Grid (', interpolationMethod, ')']);

    % --- Plot 2: Raw Data from S2_filtered ---
    if ismember(coeff, S2_filtered.Properties.VariableNames)
        mach_S2 = S2_filtered.Mach;
        alpha_S2 = S2_filtered.Alpha;
        coeff_S2 = S2_filtered.(coeff);
        if strcmp(coeff, 'CP')
            coeff_S2 = coeff_S2 * inch_to_meter;
        end
        scatter3(mach_S2, alpha_S2, coeff_S2, markerSizeRaw, colorS2, 'filled', ...
                 'DisplayName', 'Source: 0-4 deg Sheet');
    else
         fprintf('  Skipping raw S2 plot for %s (not found in S2_filtered).\n', coeff);
    end

    % --- Plot 3: Raw Data from S3_filtered ---
    if ismember(coeff, S3_filtered.Properties.VariableNames)
        mach_S3 = S3_filtered.Mach;
        alpha_S3 = S3_filtered.Alpha;
        coeff_S3 = S3_filtered.(coeff);
        if strcmp(coeff, 'CP')
            coeff_S3 = coeff_S3 * inch_to_meter;
        end
        scatter3(mach_S3, alpha_S3, coeff_S3, markerSizeRaw, colorS3, 'filled', ...
                 'DisplayName', 'Source: 0-15 deg Sheet');
    else
        fprintf('  Skipping raw S3 plot for %s (not found in S3_filtered).\n', coeff);
    end

    xlabel('Mach');
    ylabel('Alpha (deg)');
    zlabel(zLabelString, 'Interpreter', 'tex');
    title(['Combined Plot (Method: ', interpolationMethod, '): ', coeffSafe, ' vs Mach vs Alpha'], 'Interpreter', 'tex');
    colormap(gca, 'parula');
    colorbar;
    legend('show', 'Location', 'best');
    grid on;
    view(-30, 30);
    hold off;

    fprintf('  Generated combined plot for %s.\n', coeff);
end
fprintf('All combined plots generated.\n');

fprintf('\n--- Processing Finished ---\n');

