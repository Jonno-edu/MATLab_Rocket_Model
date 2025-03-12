%% initialize.m - Initialize rocket simulation parameters
clearvars -except STeVe*; clc; close all;

%% System Parameters
Initial.Conditions.theta0 = 90;                  % Initial pitch [deg]
Initial.Conditions.V0 = 0;                       % Initial velocity [m/s]
Initial.Conditions.h0 = 0;                       % Initial altitude [m]

Actuators.Nozzle.NaturalFreq = 1;                % wn_act [rad/s]
Actuators.Nozzle.DampingRatio = 0.3;             % z_act
Actuators.Nozzle.MaxDeflection = deg2rad(30);    % maxdef_nozzle [rad]
Actuators.Nozzle.RateLimit = deg2rad(1000);      % rate_lim_nozzle [rad/s]
Actuators.Nozzle.MomentArm = 0.1;                % nozzle_moment_arm [m]

Actuators.Engine.MaxThrust = 27.6*10^3;          % max_thrust [N]
Actuators.Engine.BurnTime = 63;                  % Burn time in seconds
Actuators.Engine.DelayBeforeStart = 5;           % Delay time in seconds

%% Import Mass Data (Verify Time Spacing First)
if ~exist('STeVeV1NoFins', 'var')
    opts = detectImportOptions("STeVe V1 No Fins.xlsx", "Sheet", "Mass Properties");
    opts.DataRange = "A2:F12602";
    opts.VariableNames = ["Time", "Mass", "COM_Z", "MOIx_Z", "MOIy_Z", "MOIz_Z"];
    STeVeV1NoFins = readtable("STeVe V1 No Fins.xlsx", opts);
    
    % Verify original time spacing
    dt = diff(STeVeV1NoFins.Time);
    assert(all(abs(dt - 0.005) < 1e-10),...
        'Time vector not perfectly spaced! Max Δt error: %.2e', max(abs(dt - 0.005)));
end

%% Import Aerodynamic Coefficient Data
if ~exist('STeVeV1NoFinsS2', 'var')
    opts = spreadsheetImportOptions("NumVariables", 15);
    
    % Specify sheet and range
    opts.Sheet = "Aero Properties";
    opts.DataRange = "A2:O7501";
    
    % Specify column names and types
    opts.VariableNames = ["Mach", "Alpha", "CD", "CDPower_Off", "CDPower_ON", "CAPower_Off", "CAPower_On", "CL", "CN", "CNPotential", "CNViscous", "Cnalpha_0_4deg__perRad_", "CP", "CP_0_4deg_", "Reynolds"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];
    
    % Specify variable properties
    opts = setvaropts(opts, "Reynolds", "WhitespaceRule", "preserve");
    opts = setvaropts(opts, "Reynolds", "EmptyFieldRule", "auto");
    
    % Import the data
    STeVeV1NoFinsS2 = readtable("STeVe V1 No Fins.xlsx", opts, "UseExcel", false);
    
    % Import the high angle of attack data if needed
    opts.Sheet = "Aero Properties 15deg";
    opts.DataRange = "A2:O81";
    STeVeV1NoFinsS3 = readtable("STeVe V1 No Fins.xlsx", opts, "UseExcel", false);
end

%% Import Contour Data and Calculate Reference Area
if ~exist('STeVe_Contour', 'var')
    opts = spreadsheetImportOptions("NumVariables", 3);
    
    % Specify sheet and range
    opts.Sheet = "Sheet1";
    opts.DataRange = "A2:C50";
    
    % Specify column names and types
    opts.VariableNames = ["X", "Y", "Var3"];
    opts.VariableTypes = ["double", "double", "string"];
    
    % Specify variable properties
    opts = setvaropts(opts, "Var3", "WhitespaceRule", "preserve");
    opts = setvaropts(opts, "Var3", "EmptyFieldRule", "auto");
    
    % Import the data
    STeVe_Contour = readtable("STeVe-Contour.xlsx", opts, "UseExcel", false);
end

%% Calculate Reference Area from Contour
% Find the maximum radius (Y value) in the contour data
max_radius = max(STeVe_Contour.Y);

% Calculate diameter and reference area
RocketAeroPhysical.Diameter = 2 * max_radius;
RocketAeroPhysical.Reference_Area = pi * max_radius^2;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter;

% Calculate rocket length
rocket_length = max(STeVe_Contour.X) - min(STeVe_Contour.X);
RocketAeroPhysical.Length = rocket_length;

% Display physical parameters
fprintf('\n--- Rocket Physical Parameters ---\n');
fprintf('Diameter: %.3f m\n', RocketAeroPhysical.Diameter);
fprintf('Reference area: %.6f m²\n', RocketAeroPhysical.Reference_Area);
fprintf('Rocket length: %.3f m\n', RocketAeroPhysical.Length);

%% Process Aerodynamic Data for Lookup Tables
% Get unique Mach and Alpha values
uniqueMach = unique(STeVeV1NoFinsS2.Mach);
uniqueAlpha = unique(STeVeV1NoFinsS2.Alpha);

% Create structured data for Simulink lookup tables
AeroData.Breakpoints.Mach = uniqueMach;
AeroData.Breakpoints.Alpha = uniqueAlpha;

% Process main aerodynamic coefficients - focus on body-axis coefficients
CN_table = nan(length(uniqueAlpha), length(uniqueMach));
CAPower_Off_table = nan(length(uniqueAlpha), length(uniqueMach));
CAPower_On_table = nan(length(uniqueAlpha), length(uniqueMach));
CP_table = nan(length(uniqueAlpha), length(uniqueMach));
Cnalpha_table = nan(length(uniqueAlpha), length(uniqueMach));

% Fill tables
for i = 1:length(STeVeV1NoFinsS2.Mach)
    mIdx = find(uniqueMach == STeVeV1NoFinsS2.Mach(i));
    aIdx = find(uniqueAlpha == STeVeV1NoFinsS2.Alpha(i));
    
    CN_table(aIdx, mIdx) = STeVeV1NoFinsS2.CN(i);
    CAPower_Off_table(aIdx, mIdx) = STeVeV1NoFinsS2.CAPower_Off(i);
    CAPower_On_table(aIdx, mIdx) = STeVeV1NoFinsS2.CAPower_On(i);
    CP_table(aIdx, mIdx) = STeVeV1NoFinsS2.CP(i) * 0.0254; % Convert from Inch to Metre
    Cnalpha_table(aIdx, mIdx) = STeVeV1NoFinsS2.Cnalpha_0_4deg__perRad_(i);
end

% Store in AeroData structure
AeroData.Tables.CN = CN_table;
AeroData.Tables.CAPower_Off = CAPower_Off_table;
AeroData.Tables.CAPower_On = CAPower_On_table;
AeroData.Tables.CP = CP_table;
AeroData.Tables.Cnalpha = Cnalpha_table;

% Create separate breakpoint variables for Simulink prelookup blocks
Mach_Breakpoints = AeroData.Breakpoints.Mach;
Alpha_Breakpoints = AeroData.Breakpoints.Alpha;

%% Create Index-Based Prelookup Data for Mass Properties
% Store the time step for reference
timeStep = 0.005;

% Use indices (1, 2, 3...) as the breakpoints
numPoints = length(STeVeV1NoFins.Time);
indices = 1:numPoints;

% Convert data to single precision
MassData.Time = single(STeVeV1NoFins.Time);      % Keep original time for reference
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z);    % Z→X
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z);   % Z→X
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);   % X→Y
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);   % Y→Z... %% Enforce Axisymmetry and Calculate Derivatives
avg_moi = (MassData.MOI_Y + MassData.MOI_Z)/2;
MassData.MOI_Y = avg_moi;
MassData.MOI_Z = avg_moi;
MassData.dMdt = gradient(MassData.Mass, timeStep);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStep);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStep);
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStep);

avg_dIdt = (MassData.dIdt_Y + MassData.dIdt_Z)/2;
MassData.dIdt_Y = avg_dIdt;
MassData.dIdt_Z = avg_dIdt;

%% Create Inertia Tensor and Change-in-Inertia Tensor
% Initialize 3D arrays to store tensors at each time point
inertia_tensor = zeros(3, 3, numPoints, 'single');
d_inertia_tensor = zeros(3, 3, numPoints, 'single');

% Fill the arrays with tensor data at each time point
for i = 1:numPoints
    % Build inertia tensor (assuming axisymmetric body)
    inertia_tensor(:,:,i) = [
        MassData.MOI_X(i),  0,              0;
        0,                  MassData.MOI_Y(i), 0;
        0,                  0,              MassData.MOI_Y(i)
    ];
    
    % Build change-in-inertia tensor
    d_inertia_tensor(:,:,i) = [
        MassData.dIdt_X(i), 0,              0;
        0,                  MassData.dIdt_Y(i), 0;
        0,                  0,              MassData.dIdt_Y(i)
    ];
end

%% Create Prelookup Structure for Simulink (Index-Based)
PrelookupData.Breakpoints.Index = indices;
PrelookupData.Tables.Mass = MassData.Mass;
PrelookupData.Tables.COM_X = MassData.COM_X;
PrelookupData.Tables.MOI_X = MassData.MOI_X;
PrelookupData.Tables.MOI_Y = MassData.MOI_Y;
PrelookupData.Tables.dMdt = MassData.dMdt;
PrelookupData.Tables.dIdt_X = MassData.dIdt_X;
PrelookupData.Tables.dIdt_Y = MassData.dIdt_Y;
PrelookupData.Tables.InertialTensor = inertia_tensor;
PrelookupData.Tables.dInertialTensor = d_inertia_tensor;
PrelookupData.TimeStep = timeStep;  % Save time step for converting time to index

%% Create complete RocketAero structure
RocketAero.Physical = RocketAeroPhysical;
%% Export to Workspace and Save to MAT file
% Export to base workspace
assignin('base', 'PrelookupData', PrelookupData);
assignin('base', 'Initial', Initial);
assignin('base', 'Actuators', Actuators);
assignin('base', 'AeroData', AeroData);
assignin('base', 'RocketAero', RocketAero);
assignin('base', 'Mach_Breakpoints', Mach_Breakpoints);
assignin('base', 'Alpha_Breakpoints', Alpha_Breakpoints);
assignin('base', 'Ref_Area', RocketAeroPhysical.Reference_Area);

% Save to MAT file for use with the AerodynamicsSystem class
save('RocketSimData.mat', 'PrelookupData', 'Initial', 'Actuators', 'AeroData', 'RocketAero',...
    'Mach_Breakpoints', 'Alpha_Breakpoints', 'Ref_Area');

%% Final Validation Message
fprintf('\n--- Initialization Complete ---\n');
disp('Exported Mass Properties Data to workspace and saved to RocketSimData.mat');
fprintf('Time step: %.4f seconds\n', timeStep);
fprintf('Exported Aerodynamic Data to workspace.\n');
disp('Use AerodynamicsSystem class to calculate forces and moments during simulation.');
