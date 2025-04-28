function [Initial, Actuators, RocketAero, PrelookupData, Mach_Breakpoints, Alpha_Breakpoints, Ref_Area, Wind, Sim, AeroData, RocketAeroPhysical, rocket_length, timeStep] = initialize_parameters_func()
%initialize_parameters_func - Initialize rocket simulation parameters as a function.
%   Outputs:
%       Initial: Structure with initial conditions.
%       Actuators: Structure with actuator parameters.
%       RocketAero: Structure containing ONLY physical data (RocketAero.Physical).
%       PrelookupData: Structure with mass properties for index-based lookup.
%       Mach_Breakpoints: Vector of Mach number breakpoints for aero lookup.
%       Alpha_Breakpoints: Vector of Angle of Attack breakpoints for aero lookup.
%       Ref_Area: Rocket reference area.
%       Wind: Structure with wind parameters.
%       Sim: Structure with simulation time parameters.
%       AeroData: Structure with aerodynamic coefficient tables and breakpoints (as loaded).
%       RocketAeroPhysical: Structure with physical dimensions (Diameter, Length, Ref_Area, Ref_Length).
%       rocket_length: Overall length of the rocket.
%       timeStep: Time step used for mass property data (scalar).

%% System Parameters
Sim.Timestep = 0.0001;
Sim.Time = 63;

Initial.Conditions.theta0 = deg2rad(90);         % Initial pitch [deg]
Initial.Conditions.tiltAngle = deg2rad(75);         % Initial pitch [deg]
Initial.Conditions.pitchRate = (0.5);      % Default Pitch rate [deg/s] - will be overridden in sweep
Initial.Conditions.V0 = 0;                       % Initial velocity [m/s]
Initial.Conditions.h0 = (-1)*0;                  % Initial altitude [m]

Wind.shear = 0.1;                                 % Wind shear [m/s]

Actuators.Nozzle.NaturalFreq = 1000;                % wn_act [rad/s]
Actuators.Nozzle.DampingRatio = 0.707;             % z_act
Actuators.Nozzle.MaxDeflection = deg2rad(8);    % maxdef_nozzle [rad]
Actuators.Nozzle.RateLimit = deg2rad(150);      % rate_lim_nozzle [rad/s]

Actuators.Engine.MaxThrust = (27.6*10^3)*1;          % max_thrust [N]
% Burn Time calculated in mass data section below

%Actuators.Engine.DelayBeforeStart = 5;           % Delay time in seconds

%% Set up file paths
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
% Path to Excel for Mass and Contour data
massDataFilePath = fullfile(projectRoot, 'data', 'aero', 'STeVe V1 No Fins.xlsx'); % Renamed for clarity
contourFilePath = fullfile(projectRoot, 'data', 'parameters', 'STeVe-Contour.xlsx');
% Path to the pre-processed Aero MAT file
% Using absolute path as it was in the original script - consider making relative
aeroMatFilePath = fullfile('/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data', 'CombinedAeroData_Grid.mat');

%% Import Mass Data (Verify Time Spacing First)
% Load from file every time function is called, or consider persistent variables/passing data
fprintf('Loading Mass data from Excel for function call...\\n');
opts = detectImportOptions(massDataFilePath, "Sheet", "Mass Properties");
opts.DataRange = "A2:F12602";
opts.VariableNames = ["Time", "Mass", "COM_Z", "MOIx_Z", "MOIy_Z", "MOIz_Z"];
STeVeV1NoFins = readtable(massDataFilePath, opts);

% Verify original time spacing
dt = diff(STeVeV1NoFins.Time);
if ~all(abs(dt - 0.005) < 1e-10)
    warning('Time vector not perfectly spaced! Max Δt error: %.2e', max(abs(dt - 0.005)));
end
fprintf('Mass data loaded.\\n');


% --------------------------------------------------------------------------
%% Load Pre-Processed Aerodynamic Data from MAT file
% --------------------------------------------------------------------------
fprintf('Loading pre-processed aerodynamic data from: %s\\n', aeroMatFilePath);
if exist(aeroMatFilePath, 'file')
    loadedData = load(aeroMatFilePath, 'CombinedAeroData'); % Load the specific structure
    AeroData = loadedData.CombinedAeroData; % Assign to the expected structure name
    fprintf('Pre-processed aerodynamic data loaded successfully.\\n');
    fprintf('  Interpolation method used: %s\\n', AeroData.Info.InterpolationMethod);

    % Create separate breakpoint variables for Simulink prelookup blocks (as done previously)
    Mach_Breakpoints = AeroData.Breakpoints.Mach;
    Alpha_Breakpoints = AeroData.Breakpoints.Alpha;
    fprintf('  Mach breakpoints range: %.2f to %.2f (%d points)\\n', min(Mach_Breakpoints), max(Mach_Breakpoints), length(Mach_Breakpoints));
    fprintf('  Alpha breakpoints range: %.2f to %.2f (%d points)\\n', min(Alpha_Breakpoints), max(Alpha_Breakpoints), length(Alpha_Breakpoints));

    % Rename Cnalpha field for consistency
    if isfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_')
        AeroData.Tables.Cnalpha = AeroData.Tables.Cnalpha_0_4deg__perRad_;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_');
        fprintf('  Renamed AeroData.Tables.Cnalpha_0_4deg__perRad_ to AeroData.Tables.Cnalpha for compatibility.\\n');
    end

    % Verify expected tables are present
    expectedTables = {'CN', 'CAPower_On', 'CP', 'Cnalpha'};
    foundTables = fieldnames(AeroData.Tables);
    if ~all(ismember(expectedTables, foundTables))
         warning('Loaded AeroData is missing expected tables.');
    end
    if isfield(AeroData.Tables, 'CAPower_Off')
         warning('Loaded AeroData unexpectedly contains CAPower_Off. It will be ignored.');
         AeroData.Tables = rmfield(AeroData.Tables, 'CAPower_Off');
    end
    fprintf('  AeroData structure contains tables: %s\\n', strjoin(fieldnames(AeroData.Tables), ', '));

else
    error('Aerodynamic MAT file not found: %s\\nPlease run the data processing script first.', aeroMatFilePath);
end
% ------------------- End of Aero Data Loading ---------------------------

%% Import Contour Data and Calculate Reference Area
fprintf('Loading Contour data from Excel for function call...\\n');
opts = spreadsheetImportOptions("NumVariables", 3);
opts.Sheet = "Sheet1";
opts.DataRange = "A2:C50";
opts.VariableNames = ["X", "Y", "Var3"];
opts.VariableTypes = ["double", "double", "string"];
opts = setvaropts(opts, "Var3", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Var3", "EmptyFieldRule", "auto");
STeVe_Contour = readtable(contourFilePath, opts, "UseExcel", false);
fprintf('Contour data loaded.\\n');

%% Calculate Reference Area from Contour
max_radius = max(STeVe_Contour.Y);
RocketAeroPhysical.Diameter = 2 * max_radius;
RocketAeroPhysical.Reference_Area = pi * max_radius^2;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter; % Assuming ref length = diameter
rocket_length = max(STeVe_Contour.X) - min(STeVe_Contour.X);
RocketAeroPhysical.Length = rocket_length;

% Assign Ref_Area output variable
Ref_Area = RocketAeroPhysical.Reference_Area;

fprintf('\\n--- Rocket Physical Parameters ---\\n');
fprintf('Diameter: %.3f m\\n', RocketAeroPhysical.Diameter);
fprintf('Reference area: %.6f m^2\\n', RocketAeroPhysical.Reference_Area);
fprintf('Rocket length: %.3f m\\n', RocketAeroPhysical.Length);

%% Create Index-Based Prelookup Data for Mass Properties
fprintf('\\nProcessing Mass data for lookup...\\n');
timeStep = 0.005; % As verified during mass data import
numPoints = height(STeVeV1NoFins);
indices = 1:numPoints;

MassData.Time = single(STeVeV1NoFins.Time);
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z);    % Z -> X body axis
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z);   % MOIz -> MOI_X body axis (roll)
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);   % MOIx -> MOI_Y body axis (pitch)
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);   % MOIy -> MOI_Z body axis (yaw)

%% Enforce Axisymmetry and Calculate Derivatives
fprintf('Enforcing axisymmetry and calculating mass derivatives...\\n');
avg_moi = (MassData.MOI_Y + MassData.MOI_Z) / 2;
MassData.MOI_Y = avg_moi;
MassData.MOI_Z = avg_moi;
MassData.dMdt = gradient(MassData.Mass, timeStep);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStep);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStep);
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStep);

Actuators.Engine.BurnTime = double(MassData.Time(end));
fprintf('Calculated Burn Time: %.3f s\\n', Actuators.Engine.BurnTime);

%% Create Inertia Tensor and Change-in-Inertia Tensor
fprintf('Creating inertia tensor time histories...\\n');
inertia_tensor = zeros(3, 3, numPoints, 'single');
d_inertia_tensor = zeros(3, 3, numPoints, 'single');
for i = 1:numPoints
    inertia_tensor(:,:,i) = diag([MassData.MOI_X(i), MassData.MOI_Y(i), MassData.MOI_Z(i)]);
    d_inertia_tensor(:,:,i) = diag([MassData.dIdt_X(i), MassData.dIdt_Y(i), MassData.dIdt_Z(i)]);
end

%% Create Prelookup Structure for Simulink (Index-Based)
fprintf('Creating index-based PrelookupData structure...\\n');
PrelookupData.Breakpoints.Index = single(indices);
PrelookupData.Tables.Mass = MassData.Mass;
PrelookupData.Tables.COM_X = MassData.COM_X;
PrelookupData.Tables.MOI_X = MassData.MOI_X;
PrelookupData.Tables.MOI_Y = MassData.MOI_Y;
PrelookupData.Tables.dMdt = MassData.dMdt;
PrelookupData.Tables.dIdt_X = MassData.dIdt_X;
PrelookupData.Tables.dIdt_Y = MassData.dIdt_Y;
PrelookupData.Tables.MOI_Z = MassData.MOI_Z; % Include MOI_Z
PrelookupData.Tables.dIdt_Z = MassData.dIdt_Z; % Include dIdt_Z
PrelookupData.Tables.InertialTensor = inertia_tensor;
PrelookupData.Tables.dInertialTensor = d_inertia_tensor;
PrelookupData.TimeStep = timeStep;

%% Create complete RocketAero structure (Physical only)
RocketAero.Physical = RocketAeroPhysical;
% RocketAero.AeroData = AeroData; % DO NOT nest AeroData here for parsim compatibility

fprintf('\\n--- Initialization Function Complete ---\\n');
% Outputs are returned automatically by the function definition
% AeroData (loaded) and RocketAeroPhysical are now separate outputs

end % End of function initialize_parameters_func
