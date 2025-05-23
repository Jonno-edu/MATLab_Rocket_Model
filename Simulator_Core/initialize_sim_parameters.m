function sim_params = initialize_sim_parameters()
% initialize_sim_parameters - Initializes all parameters for the STEVE rocket simulation.
% Outputs:
%   sim_params: A structure containing all necessary simulation parameters.

fprintf('--- Initializing Simulation Parameters ---\n');

% Define project root assuming this script is in Simulator_Core
projectRoot = fileparts(fileparts(mfilename('fullpath'))); % Goes up two levels from Simulator_Core

%% System Parameters
Sim.Timestep = 0.0001;
Sim.Time = 65;
sim_params.Sim = Sim;

Initial.Conditions.theta0 = deg2rad(80);         % Initial pitch [deg]
Initial.Conditions.tiltAngle = deg2rad(74.5);    % Initial pitch [deg]
Initial.Conditions.pitchRate = (0.49);           % Pitch rate [deg/s]
Initial.Conditions.V0 = 0;                       % Initial velocity [m/s]
Initial.Conditions.h0 = (-1)*0;                       % Initial altitude [m] (positive up)
sim_params.Initial = Initial;

Actuators.Engine.BurnTime = 60;
Actuators.Nozzle.MaxDeflection = deg2rad(8);     % maxdef_nozzle [rad]
Actuators.Nozzle.RateLimit = deg2rad(150);       % rate_lim_nozzle [rad/s]
Actuators.Engine.MaxThrust = (27.6*10^3)*1;      % max_thrust [N]
sim_params.Actuators = Actuators;

Wind.shear = 0.1;                                % Wind shear [m/s]
sim_params.Wind = Wind;

%% File Paths (relative to projectRoot)
% Input data for the simulator core
massDataFilePath      = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'mass_properties.xlsx');
contourFilePath       = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'rocket_geometry.xlsx');
aeroMatFilePath       = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'aero_lookup_data.mat');
thrustDataFilePath    = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'thrust_curve.xlsx');
controllerConfigPath  = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'controller_config.mat'); % For loading controller gains

% Output data path
sim_params.OutputDataPath = fullfile(projectRoot, 'Simulator_Core', 'output_data');

%% Load Controller Configuration (if it exists)
if exist(controllerConfigPath, 'file')
    fprintf('Loading controller configuration from: %s\n', controllerConfigPath);
    load(controllerConfigPath, 'Controller'); % Assuming gains are saved in a struct 'Controller'
    sim_params.Controller = Controller;
    fprintf('Controller configuration loaded.\n');
else
    warning('Controller configuration file not found: %s. Using default/no control gains.', controllerConfigPath);
    sim_params.Controller = struct(); % Empty struct or default values
end

%% Import Mass Data
fprintf('Loading Mass data from Excel: %s\n', massDataFilePath);
if exist(massDataFilePath, 'file')
    opts = detectImportOptions(massDataFilePath, "Sheet", "Mass Properties");
    opts.DataRange = "A2:F12602"; % Ensure this range is still correct
    opts.VariableNames = ["Time", "Mass", "COM_Z", "MOIx_Z", "MOIy_Z", "MOIz_Z"];
    STeVeV1NoFins = readtable(massDataFilePath, opts);
    dt_mass = diff(STeVeV1NoFins.Time);
    assert(all(abs(dt_mass - 0.005) < 1e-10), 'Mass data time vector not perfectly spaced!');
    fprintf('Mass data loaded.\n');
else
    error('Mass data file not found: %s', massDataFilePath);
end

%% Load Pre-Processed Aerodynamic Data from MAT file
fprintf('Loading pre-processed aerodynamic data from: %s\n', aeroMatFilePath);
if exist(aeroMatFilePath, 'file')
    load(aeroMatFilePath, 'CombinedAeroData'); % Expecting 'CombinedAeroData'
    AeroData = CombinedAeroData;
    fprintf('Pre-processed aerodynamic data loaded successfully.\n');
    fprintf('  Interpolation method used: %s\n', AeroData.Info.InterpolationMethod);
    Mach_Breakpoints = AeroData.Breakpoints.Mach;
    Alpha_Breakpoints = AeroData.Breakpoints.Alpha;
    fprintf('  Mach breakpoints range: %.2f to %.2f (%d points)\n', min(Mach_Breakpoints), max(Mach_Breakpoints), length(Mach_Breakpoints));
    fprintf('  Alpha breakpoints range: %.2f to %.2f (%d points)\n', min(Alpha_Breakpoints), max(Alpha_Breakpoints), length(Alpha_Breakpoints));
    if isfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_') % Compatibility
        AeroData.Tables.Cnalpha = AeroData.Tables.Cnalpha_0_4deg__perRad_;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_');
    end
else
    error('Aerodynamic MAT file not found: %s', aeroMatFilePath);
end

%% Import Contour Data and Calculate Reference Area
fprintf('Loading Contour data from Excel: %s\n', contourFilePath);
if exist(contourFilePath, 'file')
    opts_contour = spreadsheetImportOptions("NumVariables", 3);
    opts_contour.Sheet = "Sheet1"; % Ensure sheet name is correct
    opts_contour.DataRange = "A2:C50"; % Ensure range is correct
    opts_contour.VariableNames = ["X", "Y", "Var3"];
    opts_contour.VariableTypes = ["double", "double", "string"];
    STeVe_Contour = readtable(contourFilePath, opts_contour, "UseExcel", false);
    fprintf('Contour data loaded.\n');
else
    error('Contour data file not found: %s', contourFilePath);
end

max_radius = max(STeVe_Contour.Y);
RocketAeroPhysical.Diameter = 2 * max_radius;
RocketAeroPhysical.Reference_Area = pi * max_radius^2;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter;
RocketAeroPhysical.Length = max(STeVe_Contour.X) - min(STeVe_Contour.X);
fprintf('\n--- Rocket Physical Parameters ---\n');
fprintf('Diameter: %.3f m, Reference Area: %.6f m^2, Length: %.3f m\n', ...
    RocketAeroPhysical.Diameter, RocketAeroPhysical.Reference_Area, RocketAeroPhysical.Length);

%% Process Mass Data for Lookup
fprintf('\nProcessing Mass data for lookup...\n');
timeStepMass = 0.005; % From mass data properties
numPoints = height(STeVeV1NoFins);
indices = 1:numPoints;

MassData.Time = single(STeVeV1NoFins.Time);
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z); % Z from file -> X body axis
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z);% MOIz from file -> MOI_X body axis (roll)
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);% MOIx from file -> MOI_Y body axis (pitch)
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);% MOIy from file -> MOI_Z body axis (yaw)

avg_moi_lat = (MassData.MOI_Y + MassData.MOI_Z) / 2; % Enforce axisymmetry for pitch/yaw MOI
MassData.MOI_Y = avg_moi_lat;
MassData.MOI_Z = avg_moi_lat;

MassData.dMdt = gradient(MassData.Mass, timeStepMass);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStepMass);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStepMass);
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStepMass); % Same as dIdt_Y due to averaging

fprintf('Calculated Burn Time (from Actuators struct): %.3f s\n', sim_params.Actuators.Engine.BurnTime);

inertia_tensor = zeros(3, 3, numPoints, 'single');
d_inertia_tensor = zeros(3, 3, numPoints, 'single');
for i = 1:numPoints
    inertia_tensor(:,:,i) = diag([MassData.MOI_X(i), MassData.MOI_Y(i), MassData.MOI_Z(i)]);
    d_inertia_tensor(:,:,i) = diag([MassData.dIdt_X(i), MassData.dIdt_Y(i), MassData.dIdt_Z(i)]);
end

PrelookupData.Breakpoints.Index = single(indices);
PrelookupData.Tables.Mass = MassData.Mass;
PrelookupData.Tables.COM_X = MassData.COM_X;
PrelookupData.Tables.MOI_X = MassData.MOI_X;
PrelookupData.Tables.MOI_Y = MassData.MOI_Y;
PrelookupData.Tables.MOI_Z = MassData.MOI_Z;
PrelookupData.Tables.dMdt = MassData.dMdt;
PrelookupData.Tables.dIdt_X = MassData.dIdt_X;
PrelookupData.Tables.dIdt_Y = MassData.dIdt_Y;
PrelookupData.Tables.dIdt_Z = MassData.dIdt_Z;
PrelookupData.Tables.InertialTensor = inertia_tensor;
PrelookupData.Tables.dInertialTensor = d_inertia_tensor;
PrelookupData.TimeStep = timeStepMass;
sim_params.PrelookupData = PrelookupData;

%% Import Thrust Curve Data
fprintf('\nLoading Thrust Curve data from: %s\n', thrustDataFilePath);
if exist(thrustDataFilePath, 'file')
    opts_thrust = detectImportOptions(thrustDataFilePath);
    opts_thrust.VariableNames = {'Pressure_Pa', 'Thrust_N'};
    opts_thrust.DataRange = 'A2';
    opts_thrust.VariableTypes = {'double', 'double'};
    thrustTable = readtable(thrustDataFilePath, opts_thrust);
    thrustTable = sortrows(thrustTable, 'Pressure_Pa');
    
    ThrustLookupData.Breakpoints.Pressure = single(thrustTable.Pressure_Pa);
    ThrustLookupData.Tables.Thrust = single(thrustTable.Thrust_N);
    fprintf('  Pressure-dependent thrust curve data loaded (%d points).\n', height(thrustTable));
    sim_params.ThrustLookupData = ThrustLookupData;
else
    warning('Thrust curve Excel file not found: %s. Thrust lookup table not created.', thrustDataFilePath);
    sim_params.ThrustLookupData = struct(); % Empty struct
end

%% Finalize Structures for Output
RocketAero.Physical = RocketAeroPhysical;
RocketAero.AeroData = AeroData; % Contains Mach_Breakpoints, Alpha_Breakpoints, and Tables
sim_params.RocketAero = RocketAero;
sim_params.Mach_Breakpoints = Mach_Breakpoints; % For convenience if used directly in Simulink model
sim_params.Alpha_Breakpoints = Alpha_Breakpoints;% For convenience
sim_params.Ref_Area = RocketAeroPhysical.Reference_Area; % For convenience

%% Save RocketSimData.mat
rocketSimDataFile = fullfile(sim_params.OutputDataPath, 'RocketSimData.mat');
Ref_Area = RocketAeroPhysical.Reference_Area;  % <-- Added to fix save error
fprintf('\nSaving simulation setup data to: %s\n', rocketSimDataFile);
save(rocketSimDataFile, 'PrelookupData', 'ThrustLookupData', 'Initial', 'Actuators', 'RocketAero', ...
     'Mach_Breakpoints', 'Alpha_Breakpoints', 'Ref_Area', 'Sim', 'Wind');
% Add Controller to save if it was loaded:
if isfield(sim_params, 'Controller') && ~isempty(fieldnames(sim_params.Controller))
    save(rocketSimDataFile, 'Controller', '-append');
end

fprintf('\n--- Initialization Complete ---\n');
% Export all variables needed for Simulink and downstream scripts
assignin('base', 'Sim', Sim);
assignin('base', 'Initial', Initial);
assignin('base', 'Actuators', Actuators);
assignin('base', 'Wind', Wind);
assignin('base', 'RocketAeroPhysical', RocketAeroPhysical);
assignin('base', 'rocket_length', RocketAeroPhysical.Length);
assignin('base', 'AeroData', AeroData);
assignin('base', 'Mach_Breakpoints', Mach_Breakpoints);
assignin('base', 'Alpha_Breakpoints', Alpha_Breakpoints);
assignin('base', 'PrelookupData', PrelookupData);
assignin('base', 'ThrustLookupData', ThrustLookupData);
assignin('base', 'RocketAero', RocketAero);
assignin('base', 'Ref_Area', RocketAeroPhysical.Reference_Area);
assignin('base', 'STeVeV1NoFins', STeVeV1NoFins);
assignin('base', 'STeVe_Contour', STeVe_Contour);
assignin('base', 'MassData', MassData);
assignin('base', 'timeStep', timeStepMass);
assignin('base', 'inertia_tensor', inertia_tensor);
assignin('base', 'd_inertia_tensor', d_inertia_tensor);
% No explicit assignin('base', ...) as this is a function.
% The calling script (run_simulation.m) will receive sim_params.
end 