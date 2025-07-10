% --- Script to Combine, Interpolate (Natural Method), Plot (Gradient), and Export ---
clear; clc; close all;

%% Setup
fprintf('Loading raw aerodynamic data...\n');
excelFilePath = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/MATLAB_Toolbox/Aero_Data_Processing/raw_data_input/STeVe V1 No Fins.xlsx';

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

%% Define Constants and Parameters
coeffNames = {'CN', 'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'};
coeffNamesSafe = strrep(coeffNames, '_', '\_');
machLimit = 7.0;
inch_to_meter = 0.0254;
interpolationMethod = 'linear';
extrapolationMethod = 'nearest';

%% Filter Raw Data
S2_filtered = STeVeV1NoFinsS2(STeVeV1NoFinsS2.Mach <= machLimit, :);
S3_filtered = STeVeV1NoFinsS3(STeVeV1NoFinsS3.Mach <= machLimit, :);

%% Generate Negative Alpha Data Based on Symmetry
fprintf('\n--- Generating Negative Alpha Data (Corrected Logic) ---\n');

% Isolate ONLY the strictly positive alpha data for mirroring
S2_pos_only = S2_filtered(S2_filtered.Alpha > 0, :);
S3_pos_only = S3_filtered(S3_filtered.Alpha > 0, :);

% Create the mirrored negative-alpha tables
S2_neg = S2_pos_only;
S3_neg = S3_pos_only;
S2_neg.Alpha = -S2_neg.Alpha;
S3_neg.Alpha = -S3_neg.Alpha;

% Apply symmetry rules based on physical properties
odd_coeffs = {'CN'}; % CN is an odd function
even_coeffs = {'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'}; % CA, CP, and Cnalpha are even

% Flip signs for odd coefficients
for i = 1:length(odd_coeffs)
    coeff = odd_coeffs{i};
    if ismember(coeff, S2_neg.Properties.VariableNames), S2_neg.(coeff) = -S2_neg.(coeff); end
    if ismember(coeff, S3_neg.Properties.VariableNames), S3_neg.(coeff) = -S3_neg.(coeff); end
end

% Combine the ORIGINAL data with the NEW negative data
S2_symmetric = [S2_neg; S2_filtered];
S3_symmetric = [S3_neg; S3_filtered];
fprintf('Negative alpha data generated and combined correctly.\n\n');

%% Prepare and Combine Data for Interpolation
fprintf('Preparing and combining symmetric data for interpolation...\n');
combinedCoeffs = struct();

for i = 1:length(coeffNames)
    coeff = coeffNames{i};
    
    if strcmp(coeff, 'Cnalpha_0_4deg__perRad_')
        fprintf('  - Special handling for %s: using low-alpha data only.\n', coeff);
        combinedMach = S2_symmetric.Mach;
        combinedAlpha = S2_symmetric.Alpha;
        tempCoeffData = S2_symmetric.(coeff);
    else
        S2_for_combine = S2_symmetric(abs(S2_symmetric.Alpha) <= 4, :);
        S3_for_combine = S3_symmetric(abs(S3_symmetric.Alpha) > 4, :);
        
        combinedMach = [S2_for_combine.Mach; S3_for_combine.Mach];
        combinedAlpha = [S2_for_combine.Alpha; S3_for_combine.Alpha];
        tempCoeffData = [S2_for_combine.(coeff); S3_for_combine.(coeff)];
    end
    
    if strcmp(coeff, 'CP'), tempCoeffData = tempCoeffData * inch_to_meter; end
    
    nan_mask = isnan(tempCoeffData);
    combinedMach(nan_mask) = [];
    combinedAlpha(nan_mask) = [];
    tempCoeffData(nan_mask) = [];
    
    combinedCoeffs.(coeff).Mach = combinedMach;
    combinedCoeffs.(coeff).Alpha = combinedAlpha;
    combinedCoeffs.(coeff).Value = tempCoeffData;
end
fprintf('Data preparation complete.\n');

%% Define and Interpolate onto Target Grid
% --- THIS SECTION IS CORRECTED ---
numMachPoints = 60;
numAlphaPoints = 50;

% Define Mach grid from the full range of S2 (low alpha sheet covers full Mach range)
targetMach = linspace(min(S2_symmetric.Mach), max(S2_symmetric.Mach), numMachPoints);

% Define Alpha grid from the combined range of BOTH sheets
fullAlphaRange = [S2_symmetric.Alpha; S3_symmetric.Alpha];
targetAlpha = linspace(min(fullAlphaRange), max(fullAlphaRange), numAlphaPoints);

[MachGrid, AlphaGrid] = meshgrid(targetMach, targetAlpha);
fprintf('Target grid defined over full alpha range (%.2f to %.2f deg).\n', min(targetAlpha), max(targetAlpha));
% --- END OF CORRECTION ---


interpolatedTables = struct();
for i = 1:length(coeffNames)
    coeff = coeffNames{i};
    fprintf('  Interpolating %s...\n', coeff);
    
    sourceMach = combinedCoeffs.(coeff).Mach;
    sourceAlpha = combinedCoeffs.(coeff).Alpha;
    sourceValue = combinedCoeffs.(coeff).Value;
    
    [uniqueMA, ia, ~] = unique([sourceMach, sourceAlpha], 'rows', 'stable');
    
    F_interp = scatteredInterpolant(uniqueMA(:,1), uniqueMA(:,2), sourceValue(ia), interpolationMethod, extrapolationMethod);
    interpolatedTables.(coeff) = F_interp(MachGrid, AlphaGrid);
    fprintf('  Done.\n');
end
fprintf('Interpolation complete.\n');

%% Package and Export Data
CombinedAeroData = struct();
CombinedAeroData.Breakpoints.Mach = targetMach(:)';
CombinedAeroData.Breakpoints.Alpha = targetAlpha(:)';
finalCoeffNames = fieldnames(interpolatedTables);
for i = 1:length(finalCoeffNames)
     coeff = finalCoeffNames{i};
     CombinedAeroData.Tables.(coeff) = interpolatedTables.(coeff);
end

targetFolder = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/MATLAB_Toolbox/Aero_Data_Processing/generated_aero_data';
outputFileName = fullfile(targetFolder, 'CombinedAeroData_Grid_Symmetric_Corrected.mat');
save(outputFileName, 'CombinedAeroData');
fprintf('Data saved successfully to %s.\n', outputFileName);

%% Visualization
fprintf('Generating combined scatter plots...\n');
for i = 1:length(finalCoeffNames)
    coeff = finalCoeffNames{i};
    coeffSafe = strrep(coeff, '_', '\_');

    figure;
    hold on;

    coeffDataGrid = CombinedAeroData.Tables.(coeff);
    scatter3(MachGrid(:), AlphaGrid(:), coeffDataGrid(:), 30, coeffDataGrid(:), 'o', 'filled', 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Interpolated Grid');

    rawMach = combinedCoeffs.(coeff).Mach;
    rawAlpha = combinedCoeffs.(coeff).Alpha;
    rawValue = combinedCoeffs.(coeff).Value;
    scatter3(rawMach, rawAlpha, rawValue, 20, 'red', 'filled', 'DisplayName', 'Source Data Points');
    
    xlabel('Mach');
    ylabel('Alpha (deg)');
    zlabel(coeffSafe, 'Interpreter', 'tex');
    title(['Combined Plot: ', coeffSafe], 'Interpreter', 'tex');
    
    colormap(gca, 'parula');
    colorbar;
    legend('show', 'Location', 'best');
    grid on;
    view(-30, 30);
    hold off;
    fprintf('  Generated combined plot for %s.\n', coeff);
end
fprintf('All combined plots generated.\n\n--- Processing Finished ---\n');
