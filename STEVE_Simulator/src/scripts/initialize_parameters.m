%% initialize.m - Initialize rocket simulation parameters
%clearvars -except STeVe*; clc; close all;

%% System Parameters
Sim.Timestep = 0.0001;
Sim.Time = 60*5;

Initial.Conditions.theta0 = deg2rad(90);         % Initial pitch [deg]
Initial.Conditions.tiltAngle = deg2rad(75);         % Initial pitch [deg]
Initial.Conditions.pitchRate = (0.55);      % Pitch rate [deg/s]
Initial.Conditions.V0 = 0;                       % Initial velocity [m/s]
Initial.Conditions.h0 = (-1)*0;                  % Initial altitude [m]

Wind.shear = 0.1;                                 % Wind shear [m/s]

Actuators.Nozzle.NaturalFreq = 1000;                % wn_act [rad/s]
Actuators.Nozzle.DampingRatio = 0.707;             % z_act
Actuators.Nozzle.MaxDeflection = deg2rad(8);    % maxdef_nozzle [rad]
Actuators.Nozzle.RateLimit = deg2rad(150);      % rate_lim_nozzle [rad/s]

Actuators.Engine.MaxThrust = (27.6*10^3)*1;          % max_thrust [N]
% Burn Time calculated in mass data section

%Actuators.Engine.DelayBeforeStart = 5;           % Delay time in seconds

%% Set up file paths
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
% Path to Excel for Mass and Contour data
massDataFilePath = fullfile(projectRoot, 'data', 'aero', 'STeVe V1 No Fins.xlsx'); % Renamed for clarity
contourFilePath = fullfile(projectRoot, 'data', 'parameters', 'STeVe-Contour.xlsx');
% Path to the pre-processed Aero MAT file
aeroMatFilePath = fullfile('/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data', 'CombinedAeroData_Grid.mat');

%% Import Mass Data (Verify Time Spacing First)
if ~exist('STeVeV1NoFins', 'var')
    fprintf('Loading Mass data from Excel...\n');
    opts = detectImportOptions(massDataFilePath, "Sheet", "Mass Properties");
    opts.DataRange = "A2:F12602";
    opts.VariableNames = ["Time", "Mass", "COM_Z", "MOIx_Z", "MOIy_Z", "MOIz_Z"];
    STeVeV1NoFins = readtable(massDataFilePath, opts);

    % Verify original time spacing
    dt = diff(STeVeV1NoFins.Time);
    assert(all(abs(dt - 0.005) < 1e-10),...
        'Time vector not perfectly spaced! Max Δt error: %.2e', max(abs(dt - 0.005)));
    fprintf('Mass data loaded.\n');
else
    fprintf('Using existing Mass data (STeVeV1NoFins) from workspace.\n');
end

% --------------------------------------------------------------------------
%% Load Pre-Processed Aerodynamic Data from MAT file
% This replaces the sections "Import Aerodynamic Coefficient Data" and
% "Process Aerodynamic Data for Lookup Tables" from the original script.
% --------------------------------------------------------------------------
fprintf('Loading pre-processed aerodynamic data from: %s\n', aeroMatFilePath);
if exist(aeroMatFilePath, 'file')
    load(aeroMatFilePath, 'CombinedAeroData'); % Load the specific structure
    AeroData = CombinedAeroData; % Assign to the expected structure name
    fprintf('Pre-processed aerodynamic data loaded successfully.\n');
    fprintf('  Interpolation method used: %s\n', AeroData.Info.InterpolationMethod);

    % Create separate breakpoint variables for Simulink prelookup blocks (as done previously)
    Mach_Breakpoints = AeroData.Breakpoints.Mach;
    Alpha_Breakpoints = AeroData.Breakpoints.Alpha;
    fprintf('  Mach breakpoints range: %.2f to %.2f (%d points)\n', min(Mach_Breakpoints), max(Mach_Breakpoints), length(Mach_Breakpoints));
    fprintf('  Alpha breakpoints range: %.2f to %.2f (%d points)\n', min(Alpha_Breakpoints), max(Alpha_Breakpoints), length(Alpha_Breakpoints));

    % Rename Cnalpha field for consistency with original save format if needed
    % The MAT file saves 'Cnalpha_0_4deg__perRad_', original script used 'Cnalpha'
    if isfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_')
        AeroData.Tables.Cnalpha = AeroData.Tables.Cnalpha_0_4deg__perRad_;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_');
        fprintf('  Renamed AeroData.Tables.Cnalpha_0_4deg__perRad_ to AeroData.Tables.Cnalpha for compatibility.\n');
    end

    % Verify expected tables are present (excluding CAPower_Off)
    expectedTables = {'CN', 'CAPower_On', 'CP', 'Cnalpha'};
    foundTables = fieldnames(AeroData.Tables);
    assert(all(ismember(expectedTables, foundTables)), 'Loaded AeroData is missing expected tables.');
    if isfield(AeroData.Tables, 'CAPower_Off')
         warning('Loaded AeroData unexpectedly contains CAPower_Off. It will be ignored.');
         AeroData.Tables = rmfield(AeroData.Tables, 'CAPower_Off');
    end
    fprintf('  AeroData structure contains tables: %s\n', strjoin(fieldnames(AeroData.Tables), ', '));

else
    error('Aerodynamic MAT file not found: %s\nPlease run the data processing script first.', aeroMatFilePath);
end
% ------------------- End of Aero Data Loading ---------------------------

%% Import Contour Data and Calculate Reference Area
if ~exist('STeVe_Contour', 'var')
    fprintf('Loading Contour data from Excel...\n');
    opts = spreadsheetImportOptions("NumVariables", 3);
    opts.Sheet = "Sheet1";
    opts.DataRange = "A2:C50";
    opts.VariableNames = ["X", "Y", "Var3"];
    opts.VariableTypes = ["double", "double", "string"];
    opts = setvaropts(opts, "Var3", "WhitespaceRule", "preserve");
    opts = setvaropts(opts, "Var3", "EmptyFieldRule", "auto");
    STeVe_Contour = readtable(contourFilePath, opts, "UseExcel", false);
    fprintf('Contour data loaded.\n');
else
     fprintf('Using existing Contour data (STeVe_Contour) from workspace.\n');
end

%% Calculate Reference Area from Contour
% Find the maximum radius (Y value) in the contour data
max_radius = max(STeVe_Contour.Y);

% Calculate diameter and reference area
RocketAeroPhysical.Diameter = 2 * max_radius;
RocketAeroPhysical.Reference_Area = pi * max_radius^2;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter; % Assuming ref length = diameter

% Calculate rocket length
rocket_length = max(STeVe_Contour.X) - min(STeVe_Contour.X);
RocketAeroPhysical.Length = rocket_length;

% Display physical parameters
fprintf('\n--- Rocket Physical Parameters ---\n');
fprintf('Diameter: %.3f m\n', RocketAeroPhysical.Diameter);
fprintf('Reference area: %.6f m^2\n', RocketAeroPhysical.Reference_Area); % Corrected superscript
fprintf('Rocket length: %.3f m\n', RocketAeroPhysical.Length);

%% Create Index-Based Prelookup Data for Mass Properties
fprintf('\nProcessing Mass data for lookup...\n');
% Store the time step for reference
timeStep = 0.005; % As verified during mass data import

% Use indices (1, 2, 3...) as the breakpoints
numPoints = height(STeVeV1NoFins); % Use height for table rows
indices = 1:numPoints;

% Convert data to single precision and handle axis mapping
MassData.Time = single(STeVeV1NoFins.Time);      % Keep original time for reference
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z);    % Z from file -> X body axis
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z);   % MOIz from file -> MOI_X body axis (roll)
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);   % MOIx from file -> MOI_Y body axis (pitch)
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);   % MOIy from file -> MOI_Z body axis (yaw)

%% Enforce Axisymmetry and Calculate Derivatives
fprintf('Enforcing axisymmetry and calculating mass derivatives...\n');
avg_moi = (MassData.MOI_Y + MassData.MOI_Z) / 2;
MassData.MOI_Y = avg_moi;
MassData.MOI_Z = avg_moi;
MassData.dMdt = gradient(MassData.Mass, timeStep);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStep);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStep); % Derivative of averaged MOI_Y
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStep); % Derivative of averaged MOI_Z

% No need to average dIdt again as MOI_Y and MOI_Z were already averaged

Actuators.Engine.BurnTime = double(MassData.Time(end));    % Burn time in seconds
fprintf('Calculated Burn Time: %.3f s\n', Actuators.Engine.BurnTime);

%% Create Inertia Tensor and Change-in-Inertia Tensor
fprintf('Creating inertia tensor time histories...\n');
% Initialize 3D arrays to store tensors at each time point
inertia_tensor = zeros(3, 3, numPoints, 'single');
d_inertia_tensor = zeros(3, 3, numPoints, 'single');

% Fill the arrays with tensor data at each time point
for i = 1:numPoints
    % Build inertia tensor (axisymmetric)
    inertia_tensor(:,:,i) = diag([MassData.MOI_X(i), MassData.MOI_Y(i), MassData.MOI_Z(i)]);
    % Build change-in-inertia tensor
    d_inertia_tensor(:,:,i) = diag([MassData.dIdt_X(i), MassData.dIdt_Y(i), MassData.dIdt_Z(i)]);
end

%% Create Prelookup Structure for Simulink (Index-Based)
fprintf('Creating index-based PrelookupData structure...\n');
PrelookupData.Breakpoints.Index = single(indices); % Use single for consistency
PrelookupData.Tables.Mass = MassData.Mass;
PrelookupData.Tables.COM_X = MassData.COM_X;
PrelookupData.Tables.MOI_X = MassData.MOI_X;
PrelookupData.Tables.MOI_Y = MassData.MOI_Y;
PrelookupData.Tables.dMdt = MassData.dMdt;
PrelookupData.Tables.dIdt_X = MassData.dIdt_X;
PrelookupData.Tables.dIdt_Y = MassData.dIdt_Y;
% Include MOI_Z and dIdt_Z if needed by model, otherwise keep consistent
PrelookupData.Tables.MOI_Z = MassData.MOI_Z;
PrelookupData.Tables.dIdt_Z = MassData.dIdt_Z;
PrelookupData.Tables.InertialTensor = inertia_tensor;
PrelookupData.Tables.dInertialTensor = d_inertia_tensor;
PrelookupData.TimeStep = timeStep;  % Save time step for converting time to index

%% Create complete RocketAero structure
RocketAero.Physical = RocketAeroPhysical;
RocketAero.AeroData = AeroData; % Add the loaded AeroData here

%% Export to Workspace and Save to MAT file
fprintf('\nExporting data to workspace and saving RocketSimData.mat...\n');
% Export to base workspace
assignin('base', 'PrelookupData', PrelookupData);
assignin('base', 'Initial', Initial);
assignin('base', 'Actuators', Actuators);
% assignin('base', 'AeroData', AeroData); % Redundant if RocketAero is used
assignin('base', 'RocketAero', RocketAero);
assignin('base', 'Mach_Breakpoints', Mach_Breakpoints);
assignin('base', 'Alpha_Breakpoints', Alpha_Breakpoints);
assignin('base', 'Ref_Area', RocketAeroPhysical.Reference_Area); % Keep for convenience if used directly

% Save to MAT file for use with the AerodynamicsSystem class or direct loading
% Note: AeroData is saved within RocketAero now. Add it explicitly if needed separately.
save('RocketSimData.mat', 'PrelookupData', 'Initial', 'Actuators', 'RocketAero', ...
    'Mach_Breakpoints', 'Alpha_Breakpoints', 'Ref_Area');

%% Final Validation Message
fprintf('\n--- Initialization Complete ---\n');
disp('Exported Simulation Data to workspace and saved relevant parts to RocketSimData.mat');
fprintf('Mass data time step: %.4f seconds\n', timeStep);
fprintf('Aerodynamic data loaded from pre-processed MAT file.\n');
