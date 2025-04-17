% --- Script to Combine, Interpolate, and Export Aerodynamic Data ---
clear; clc; close all;

fprintf('Loading raw aerodynamic data...\n');
% --- Load Raw Data (Assuming Excel file is in the specified path) ---
% Define file path (adjust as necessary)
excelFilePath = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data/STeVe V1 No Fins.xlsx'; % Or use the path logic from initialize.m

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
% Use the same variable names and types for consistency
optsS3.VariableNames = optsS2.VariableNames;
optsS3.VariableTypes = optsS2.VariableTypes;
% Adjust Reynolds type if needed (it was categorical before, ensure consistency)
optsS3.VariableTypes{15} = 'string'; % Match S2 type (Corrected syntax)
optsS3 = setvaropts(optsS3, "Reynolds", "WhitespaceRule", "preserve");
optsS3 = setvaropts(optsS3, "Reynolds", "EmptyFieldRule", "auto");
STeVeV1NoFinsS3 = readtable(excelFilePath, optsS3, "UseExcel", false);
fprintf('Raw data loaded.\n');

% --- Visualization: Combined Raw Data Scatter Plots (Color-Coded by Source) ---
% Added section starts here
fprintf('Generating scatter plots of raw combined data (color-coded by source)...\n');

% Define the coefficients required by initialize.m (and thus to plot)
% From initialize.m: CN, CAPower_Off, CAPower_On, CP, Cnalpha_0_4deg__perRad_
coeffNamesToPlot = {'CN', 'CAPower_Off', 'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'};

% Define colors for the sources
colorS2 = 'blue'; % Color for 0-4 deg sheet data
colorS3 = 'red';  % Color for 0-15 deg sheet data
markerSize = 30;  % Consistent marker size

inch_to_meter_vis = 0.0254; % For CP conversion if needed for visualization consistency

% Loop through each coefficient to create a separate plot
for i = 1:length(coeffNamesToPlot)
    coeffName = coeffNamesToPlot{i};

    % Check if the coefficient exists in both tables
    if ~ismember(coeffName, STeVeV1NoFinsS2.Properties.VariableNames) || ...
       ~ismember(coeffName, STeVeV1NoFinsS3.Properties.VariableNames)
        warning('Coefficient %s not found in both source tables. Skipping plot.', coeffName);
        continue; % Skip to the next coefficient
    end

    % Create a new figure for this coefficient
    figure;
    hold on; % Allow multiple scatter plots on the same axes

    % --- Plot data from STeVeV1NoFinsS2 (0-4 deg sheet) ---
    mach_S2 = STeVeV1NoFinsS2.Mach;
    alpha_S2 = STeVeV1NoFinsS2.Alpha;
    coeff_S2 = STeVeV1NoFinsS2.(coeffName);
    zLabelString = coeffName; % Default Z label

    % Specific handling for CP: Convert to meters for consistent plotting
    if strcmp(coeffName, 'CP')
        coeff_S2 = coeff_S2 * inch_to_meter_vis;
        zLabelString = 'CP (m)'; % Update label
    end

    scatter3(mach_S2, alpha_S2, coeff_S2, ...
             markerSize, colorS2, 'filled', ...
             'DisplayName', 'Source: 0-4 deg Sheet');

    % --- Plot data from STeVeV1NoFinsS3 (0-15 deg sheet) ---
    mach_S3 = STeVeV1NoFinsS3.Mach;
    alpha_S3 = STeVeV1NoFinsS3.Alpha;
    coeff_S3 = STeVeV1NoFinsS3.(coeffName);

    % Specific handling for CP: Convert to meters
    if strcmp(coeffName, 'CP')
        coeff_S3 = coeff_S3 * inch_to_meter_vis;
        % zLabelString is already set correctly from S2 check
    end

    scatter3(mach_S3, alpha_S3, coeff_S3, ...
             markerSize, colorS3, 'filled', ...
             'DisplayName', 'Source: 0-15 deg Sheet');

    % --- Add labels, title, legend, etc. ---
    xlabel('Mach');
    ylabel('Alpha (deg)');
    zlabel(zLabelString); % Use the potentially updated label
    title(['Raw Combined Data: ', coeffName, ' vs Mach vs Alpha']);
    legend('show', 'Location', 'best'); % Display the legend
    grid on;
    view(-30, 30); % Set consistent view
    hold off;

    fprintf('  Plotted raw combined %s.\n', coeffName);
end

fprintf('Raw combined data plots generated.\n');
% Added section ends here

% --- 1. Prepare Data for Combination ---
fprintf('Preparing and combining data...\n');

% Define the coefficients required by initialize.m (This list is now for processing)
coeffNames = {'CN', 'CAPower_Off', 'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'};

% Define Mach limit
machLimit = 7.0;

% Filter S2 (0-4 deg data)
S2_filtered = STeVeV1NoFinsS2(STeVeV1NoFinsS2.Mach <= machLimit & STeVeV1NoFinsS2.Alpha <= 4, :);

% Filter S3 (0-15 deg data) - *Only take data strictly above 4 degrees*
S3_filtered = STeVeV1NoFinsS3(STeVeV1NoFinsS3.Mach <= machLimit & STeVeV1NoFinsS3.Alpha > 4, :);

% Combine the prioritized data into single arrays
combinedMach = [S2_filtered.Mach; S3_filtered.Mach];
combinedAlpha = [S2_filtered.Alpha; S3_filtered.Alpha];

% Combine coefficients, checking existence and handling CP conversion
combinedCoeffs = struct();
inch_to_meter_proc = 0.0254; % Conversion factor for CP processing

for i = 1:length(coeffNames)
    coeff = coeffNames{i};
    if ismember(coeff, S2_filtered.Properties.VariableNames) && ismember(coeff, S3_filtered.Properties.VariableNames)
        tempCoeffData = [S2_filtered.(coeff); S3_filtered.(coeff)];

        % Specific handling for CP: Convert from inches to meters *before* interpolation
        if strcmp(coeff, 'CP')
            fprintf('  Converting CP from inches to meters for processing...\n');
            tempCoeffData = tempCoeffData * inch_to_meter_proc;
        end
        combinedCoeffs.(coeff) = tempCoeffData;
    else
        warning('Coefficient %s not found consistently in both tables for processing, skipping.', coeff);
        % Remove from list if not processed
         coeffNames = coeffNames(~strcmp(coeffNames, coeff));
         % Re-index loop if necessary (safer to just let it skip)
    end
end

% Remove duplicate (Mach, Alpha) points, keeping the first occurrence (prioritizing S2)
[uniqueMA, ia, ~] = unique([combinedMach, combinedAlpha], 'rows', 'stable');
combinedMach = uniqueMA(:, 1);
combinedAlpha = uniqueMA(:, 2);

% Update combined coefficients based on unique indices
activeCoeffNames = fieldnames(combinedCoeffs); % Get names of successfully combined coeffs
for i = 1:length(activeCoeffNames)
     coeff = activeCoeffNames{i};
     combinedCoeffs.(coeff) = combinedCoeffs.(coeff)(ia);
end
fprintf('Data preparation complete. %d unique data points.\n', length(combinedMach));

% --- 2. Define Target Grid ---
fprintf('Defining target interpolation grid...\n');
% Define the breakpoint vectors for the final lookup table
% Adjust density as needed for accuracy vs. memory/performance
numMachPoints = 60; % Example: 60 points up to Mach 7
numAlphaPoints = 50; % Example: 50 points up to 15 deg

targetMach = linspace(min(combinedMach), machLimit, numMachPoints);
% Ensure Alpha grid covers the full range, including 0 if present
minAlpha = min(combinedAlpha);
maxAlpha = max(combinedAlpha);
if minAlpha > 0
    warning('Minimum combined Alpha is %.2f, grid starts from here.', minAlpha);
end
if maxAlpha < 15 % Check against expected max, e.g., 15
     warning('Maximum combined Alpha is %.2f, grid ends here.', maxAlpha);
end
targetAlpha = linspace(minAlpha, maxAlpha, numAlphaPoints);

% Create the grid points using meshgrid
[MachGrid, AlphaGrid] = meshgrid(targetMach, targetAlpha);
fprintf('Target grid defined (Mach: %d points, Alpha: %d points).\n', numMachPoints, numAlphaPoints);

% --- 3. Interpolate onto Target Grid ---
fprintf('Interpolating coefficients onto grid...\n');
interpolatedTables = struct(); % Store final tables here
interpolationMethod = 'linear'; % Robust choice for LUTs
extrapolationMethod = 'nearest'; % Safer than linear extrapolation

for i = 1:length(activeCoeffNames)
    coeff = activeCoeffNames{i};
    fprintf('  Interpolating %s...\n', coeff);
    F_interp = scatteredInterpolant(combinedMach, combinedAlpha, combinedCoeffs.(coeff), ...
                                    interpolationMethod, extrapolationMethod);
    interpolatedTables.(coeff) = F_interp(MachGrid, AlphaGrid);
    fprintf('  Done.\n');
end
fprintf('Interpolation complete.\n');

% --- 4. Package Data for Export ---
fprintf('Packaging data for export...\n');
% Create a structure to hold the final data for Simulink
CombinedAeroData = struct();
CombinedAeroData.Breakpoints = struct();
CombinedAeroData.Tables = struct();

% Assign breakpoints (ensure they are row vectors for some Simulink conventions)
CombinedAeroData.Breakpoints.Mach = targetMach(:)'; % Ensure row vector
CombinedAeroData.Breakpoints.Alpha = targetAlpha(:)'; % Ensure row vector

% Assign interpolated tables
for i = 1:length(activeCoeffNames)
     coeff = activeCoeffNames{i};
     CombinedAeroData.Tables.(coeff) = interpolatedTables.(coeff);
end

% Add a description or version info (optional but good practice)
CombinedAeroData.Info.Description = 'Combined and interpolated aero data (0-15 deg Alpha, up to Mach 7)';
CombinedAeroData.Info.SourceFiles = {optsS2.Sheet, optsS3.Sheet}; % Store sheet names
CombinedAeroData.Info.GenerationDate = datetime('now');
CombinedAeroData.Info.InterpolationMethod = interpolationMethod;
CombinedAeroData.Info.ExtrapolationMethod = extrapolationMethod;

fprintf('Data packaged.\n');

% --- 5. Export Data to MAT File ---
targetFolder = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data';
outputFileName = fullfile(targetFolder, 'CombinedAeroData_Grid.mat'); % Construct full path

fprintf('Saving combined data to %s...\n', outputFileName);
save(outputFileName, 'CombinedAeroData'); % Save the structure
fprintf('Data saved successfully.\n');

% --- Additional Visualization: Scatter Plot for Each Interpolated Variable ---
% This section plots the FINAL GRIDDED data points

fprintf('Generating scatter plots for all interpolated variables...\n');

% Get the names of the coefficient tables that were processed and saved
coeffTableNames = fieldnames(CombinedAeroData.Tables);

% Get the breakpoint vectors (already defined as targetMach, targetAlpha)
% Reuse the meshgrid created earlier (MachGrid, AlphaGrid)

% Reshape grid coordinates into vectors for scatter3
mach_vector = MachGrid(:);
alpha_vector = AlphaGrid(:);

% Loop through each coefficient table
for i = 1:length(coeffTableNames)
    coeffName = coeffTableNames{i};
    coeffData = CombinedAeroData.Tables.(coeffName); % Get the 2D table data
    coeff_vector = coeffData(:); % Reshape into a vector

    % Create a new figure for each coefficient's interpolated grid points
    figure;

    % Create the scatter plot
    scatter3(mach_vector, alpha_vector, coeff_vector, ...
             30, ...         % Marker size
             coeff_vector, ... % Color data (use coefficient values for color)
             'filled');      % Fill the markers

    % Add labels, title, colorbar, etc.
    xlabel('Mach');
    ylabel('Alpha (deg)');
    zlabel(coeffName); % Use the coefficient name as the Z-axis label
    title(['Interpolated Grid Points: ', coeffName, ' vs Mach vs Alpha']);
    colorbar;       % Show color scale for the coefficient values
    view(-30, 30);  % Set a consistent viewing angle
    grid on;        % Add grid lines

    fprintf('  Plotted Interpolated Grid Points for %s.\n', coeffName);
end

fprintf('All interpolated data scatter plots generated.\n');

fprintf('\n--- Processing Finished ---\n');

