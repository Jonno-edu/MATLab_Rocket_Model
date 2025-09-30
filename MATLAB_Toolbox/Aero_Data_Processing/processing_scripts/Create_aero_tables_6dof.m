% --- Script to Combine, Interpolate, Normalize Names, Create Beta Duplicates, Add Cmq, Add AlphaTot Tables, and Export ---
clear; clc; close all;

%% Setup
fprintf('Loading raw aerodynamic data...\n');
% Use relative path from script location
scriptPath = mfilename('fullpath');
[scriptDir,~,~] = fileparts(scriptPath);
excelFilePath = fullfile(scriptDir, '../raw_data_input/STeVe V1 No Fins.xlsx');

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
sourceCoeffNames = {'CN', 'CAPower_On', 'CP', 'Cnalpha_0_4deg__perRad_'};
machLimit = 7.0;
inch_to_meter = 0.0254;
interpolationMethod = 'linear';
extrapolationMethod = 'nearest';

%% Filter Raw Data
S2_filtered = STeVeV1NoFinsS2(STeVeV1NoFinsS2.Mach <= machLimit, :);
S3_filtered = STeVeV1NoFinsS3(STeVeV1NoFinsS3.Mach <= machLimit, :);

%% Generate Negative Alpha Data (Symmetry)
fprintf('\n--- Generating Negative Alpha Data ---\n');
S2_pos_only = S2_filtered(S2_filtered.Alpha > 0, :);
S3_pos_only = S3_filtered(S3_filtered.Alpha > 0, :);
S2_neg = S2_pos_only; S3_neg = S3_pos_only;
S2_neg.Alpha = -S2_neg.Alpha;
S3_neg.Alpha = -S3_neg.Alpha;

% Symmetry rules
odd_coeffs  = {'CN'};
even_coeffs = {'CAPower_On','CP','Cnalpha_0_4deg__perRad_'};
for i = 1:length(odd_coeffs)
    coeff = odd_coeffs{i};
    if ismember(coeff, S2_neg.Properties.VariableNames), S2_neg.(coeff) = -S2_neg.(coeff); end
    if ismember(coeff, S3_neg.Properties.VariableNames), S3_neg.(coeff) = -S3_neg.(coeff); end
end

S2_symmetric = [S2_neg; S2_filtered];
S3_symmetric = [S3_neg; S3_filtered];
fprintf('Negative alpha data generated and combined.\n\n');

%% Prepare and Combine Data for Interpolation
fprintf('Preparing and combining symmetric data for interpolation...\n');
combinedCoeffs = struct();
for i = 1:length(sourceCoeffNames)
    coeff = sourceCoeffNames{i};

    if strcmp(coeff, 'Cnalpha_0_4deg__perRad_')
        combinedMach  = S2_symmetric.Mach;
        combinedAlpha = S2_symmetric.Alpha;
        tempCoeffData = S2_symmetric.(coeff);
    else
        S2_for_combine = S2_symmetric(abs(S2_symmetric.Alpha) <= 4, :);
        S3_for_combine = S3_symmetric(abs(S3_symmetric.Alpha) > 4, :);
        combinedMach  = [S2_for_combine.Mach;  S3_for_combine.Mach];
        combinedAlpha = [S2_for_combine.Alpha; S3_for_combine.Alpha];
        tempCoeffData = [S2_for_combine.(coeff); S3_for_combine.(coeff)];
    end

    if strcmp(coeff, 'CP'), tempCoeffData = tempCoeffData * inch_to_meter; end

    nan_mask = isnan(tempCoeffData);
    combinedMach(nan_mask)  = [];
    combinedAlpha(nan_mask) = [];
    tempCoeffData(nan_mask) = [];

    combinedCoeffs.(coeff).Mach  = combinedMach;
    combinedCoeffs.(coeff).Alpha = combinedAlpha;
    combinedCoeffs.(coeff).Value = tempCoeffData;
end
fprintf('Data preparation complete.\n');

%% Normalize Names and Create Beta Duplicates
fprintf('Normalizing names and creating beta-plane duplicates...\n');
% CAPower_On -> CA
if isfield(combinedCoeffs, 'CAPower_On')
    combinedCoeffs.CA = combinedCoeffs.CAPower_On;
    combinedCoeffs = rmfield(combinedCoeffs, 'CAPower_On');
    fprintf('  - Renamed CAPower_On to CA.\n');
end
% Cnalpha_... -> CNalpha
if isfield(combinedCoeffs, 'Cnalpha_0_4deg__perRad_')
    combinedCoeffs.CNalpha = combinedCoeffs.Cnalpha_0_4deg__perRad_;
    combinedCoeffs = rmfield(combinedCoeffs, 'Cnalpha_0_4deg__perRad_');
    fprintf('  - Renamed Cnalpha_0_4deg__perRad_ to CNalpha.\n');
end
% CY and CYbeta from CN and CNalpha (axisymmetric)
if isfield(combinedCoeffs, 'CN') && isfield(combinedCoeffs, 'CNalpha')
    combinedCoeffs.CY     = combinedCoeffs.CN;
    combinedCoeffs.CYbeta = combinedCoeffs.CNalpha;
    fprintf('  - Created CY from CN and CYbeta from CNalpha.\n');
else
    error('Cannot create beta duplicates because source CN or CNalpha is missing.');
end

%% Target Grid (ndgrid for [nMach x nAlpha])
numMachPoints  = 60;
numAlphaPoints = 50;
targetMach  = linspace(min(S2_symmetric.Mach), max(S2_symmetric.Mach), numMachPoints);
fullAlphaRange = [S2_symmetric.Alpha; S3_symmetric.Alpha];
targetAlpha = linspace(min(fullAlphaRange), max(fullAlphaRange), numAlphaPoints);
[MachGrid, AlphaGrid] = ndgrid(targetMach, targetAlpha);

%% Interpolate onto Mach×Alpha grid
finalCoeffNames = fieldnames(combinedCoeffs);
interpolatedTables = struct();
fprintf('Interpolating final coefficients onto grid...\n');
for i = 1:length(finalCoeffNames)
    coeff = finalCoeffNames{i};
    fprintf('  - Interpolating %s...\n', coeff);

    sourceMach  = combinedCoeffs.(coeff).Mach;
    sourceAlpha = combinedCoeffs.(coeff).Alpha;
    sourceValue = combinedCoeffs.(coeff).Value;

    [uniqueMA, ia, ~] = unique([sourceMach, sourceAlpha], 'rows', 'stable');
    F_interp = scatteredInterpolant(uniqueMA(:,1), uniqueMA(:,2), sourceValue(ia), interpolationMethod, extrapolationMethod);
    interpolatedTables.(coeff) = F_interp(MachGrid, AlphaGrid);
end
fprintf('Interpolation complete.\n');

%% Create Cmq (lumped) from CNalpha and mirror for beta
k_cmq = 1.0; % initial slender-body gain
if isfield(interpolatedTables, 'CNalpha')
    CNalpha_grid = interpolatedTables.CNalpha;          % [nMach x nAlpha]
    CNalpha_M    = mean(CNalpha_grid, 2);               % [nMach x 1]
    Cmq_M        = -k_cmq .* CNalpha_M;                 % [nMach x 1]
    Cmq_grid     = repmat(Cmq_M, 1, numAlphaPoints);    % [nMach x nAlpha]
    interpolatedTables.Cmq      = Cmq_grid;
    interpolatedTables.Cmq_beta = Cmq_grid;
    if ~any(strcmp(finalCoeffNames,'Cmq')),      finalCoeffNames{end+1} = 'Cmq';      end
    if ~any(strcmp(finalCoeffNames,'Cmq_beta')), finalCoeffNames{end+1} = 'Cmq_beta'; end
    fprintf('  - Created Cmq (lumped) from CNalpha and mirrored as Cmq_beta.\n');
else
    warning('CNalpha not available; Cmq could not be created.');
end

%% Add AlphaTot convention for CA and CP (alpha_total = sqrt(alpha^2 + beta^2))
% For axisymmetric bodies, CA and CP depend on total AoA; we tabulate vs AlphaTot using the Alpha grid.
AlphaTot = targetAlpha; % storage grid equals Alpha for database use
% Alias CA, CP onto AlphaTot-based names for runtime selection by alpha_total
interpolatedTables.CA_AlphaTot = interpolatedTables.CA; % same numeric table, different intended key
interpolatedTables.CP_AlphaTot = interpolatedTables.CP;
if ~any(strcmp(finalCoeffNames,'CA_AlphaTot')), finalCoeffNames{end+1} = 'CA_AlphaTot'; end
if ~any(strcmp(finalCoeffNames,'CP_AlphaTot')), finalCoeffNames{end+1} = 'CP_AlphaTot'; end
fprintf('  - Added CA_AlphaTot and CP_AlphaTot for total AoA usage.\n');

%% Package and Export Data
CombinedAeroData = struct();
CombinedAeroData.Breakpoints.Mach     = targetMach(:)';
CombinedAeroData.Breakpoints.Alpha    = targetAlpha(:)';
CombinedAeroData.Breakpoints.Beta     = targetAlpha(:)'; % Beta same as Alpha
CombinedAeroData.Breakpoints.AlphaTot = AlphaTot(:)';    % Combined AoA grid

for i = 1:length(finalCoeffNames)
     coeff = finalCoeffNames{i};
     CombinedAeroData.Tables.(coeff) = interpolatedTables.(coeff);
end

% Add metadata
CombinedAeroData.Metadata.Axisymmetry.CY_from_CN        = true;
CombinedAeroData.Metadata.Axisymmetry.CYbeta_from_CNalpha = true;
CombinedAeroData.Metadata.AoA_Combined.CA_AlphaTot_uses_total_AoA = true;
CombinedAeroData.Metadata.AoA_Combined.CP_AlphaTot_uses_total_AoA = true;

targetFolder = fullfile(scriptDir, '../generated_aero_data');
if ~exist(targetFolder, 'dir'), mkdir(targetFolder); end
outputFileName = fullfile(targetFolder, 'CombinedAeroData_6DOF.mat');
save(outputFileName, 'CombinedAeroData');
fprintf('Data saved successfully to %s.\n', outputFileName);

%% Visualization (3D scatter; shape guards)
fprintf('Generating combined scatter plots...\n');
for i = 1:length(finalCoeffNames)
    coeff = finalCoeffNames{i};
    coeffSafe = strrep(coeff, '_', '\_');

    coeffDataGrid = CombinedAeroData.Tables.(coeff);
    % Ensure table shape matches [nMach x nAlpha] grid for plot
    if ~isequal(size(coeffDataGrid), size(MachGrid))
        if isequal(size(coeffDataGrid), fliplr(size(MachGrid)))
            coeffDataGrid = coeffDataGrid.'; % transpose if swapped
        else
            % For AlphaTot, we still use AlphaGrid for visualization
            if any(strcmp(coeff, {'CA_AlphaTot','CP_AlphaTot'}))
                % Proceed with AlphaGrid; sizes should match as we aliased
            else
                warning('Skipping plot for %s: table size %s does not match grid size %s.', ...
                    coeff, mat2str(size(coeffDataGrid)), mat2str(size(MachGrid)));
                continue;
            end
        end
    end

    figure('Name', ['Combined Plot: ' coeff]);
    hold on;
    scatter3(MachGrid(:), AlphaGrid(:), coeffDataGrid(:), 30, coeffDataGrid(:), ...
        'o', 'filled', 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Interpolated Grid');

    % Overlay raw points when available (skip for synthetic tables like Cmq or AlphaTot aliases)
    if isfield(combinedCoeffs, coeff)
        rawMach  = combinedCoeffs.(coeff).Mach;
        rawAlpha = combinedCoeffs.(coeff).Alpha;
        rawValue = combinedCoeffs.(coeff).Value;
        scatter3(rawMach, rawAlpha, rawValue, 20, 'red', 'filled', 'DisplayName', 'Source Data Points');
    end

    xlabel('Mach');
    if contains(coeff, 'beta') || strcmpi(coeff, 'CY')
        ylabel('Beta (deg)');
    elseif contains(coeff, 'AlphaTot')
        ylabel('AlphaTot (deg)'); % total AoA label
    else
        ylabel('Alpha (deg)');
    end
    zlabel(coeffSafe, 'Interpreter', 'tex');
    title(['Combined Plot: ', coeffSafe], 'Interpreter', 'tex');
    colormap(gca, 'parula'); colorbar; legend('show', 'Location', 'best');
    grid on; view(-30, 30);
    hold off;
end
fprintf('All combined plots generated.\n\n--- Processing Finished ---\n');
