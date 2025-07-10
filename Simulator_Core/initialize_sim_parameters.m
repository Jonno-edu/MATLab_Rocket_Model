function sim_params = initialize_sim_parameters()
% initialize_sim_parameters - Initializes all parameters for the STEVE rocket simulation.
% Outputs:
%   sim_params: A structure containing all necessary simulation parameters.

fprintf('--- Initializing Simulation Parameters ---\n');

% Define project root assuming this script is in Simulator_Core
projectRoot = fileparts(fileparts(mfilename('fullpath'))); % Goes up two levels from Simulator_Core


%% File Paths (relative to projectRoot)
% Input data for the simulator core
heavySteveDataFilePath      = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'Heavy_STeVe_Data.xlsx');
contourFilePath       = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'rocket_geometry.xlsx');
aeroMatFilePath       = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'CombinedAeroData_Grid_Symmetric_Corrected.mat');
thrustDataFilePath    = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'thrust_curve.xlsx');
controllerConfigPath  = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'controller_config.mat'); % For loading controller gains

% Output data path
sim_params.OutputDataPath = fullfile(projectRoot, 'Simulator_Core', 'output_data');

%% Import Mass Data
fprintf('Loading Mass data from Excel: %s\n', heavySteveDataFilePath);
if exist(heavySteveDataFilePath, 'file')
    % Read the number of data points
    numsteps = readmatrix(heavySteveDataFilePath, 'Sheet', "Derived Properties", 'Range', "D4:D4");
    
    endRow = numsteps + 1;
    dynamicRange = "A2:F" + endRow;
    
    % Set up import options to read all columns, including the original time
    opts = detectImportOptions(heavySteveDataFilePath, "Sheet", "Mass Properties Heavy");
    opts.DataRange = dynamicRange;
    opts.VariableNames = ["Time", "Mass", "COM_Z", "MOIx_Z", "MOIy_Z", "MOIz_Z"];
    
    % Read the full table
    STeVeV1NoFins = readtable(heavySteveDataFilePath, opts);
    
    % Reconstruct a perfect time vector to replace the old one
    dt = 0.005; % Note: Ensure this is the correct time step for your data
    num_rows = height(STeVeV1NoFins);
    Time_new = (0:dt:(num_rows-1)*dt)';
    
    % Replace the original time column with the newly generated, perfect one
    STeVeV1NoFins.Time = Time_new;
    
    fprintf('Mass data loaded and time vector has been corrected.\n');
else
    error('Mass data file not found: %s', heavySteveDataFilePath);
end

%% System Parameters
Sim.Timestep = 0.0001;
Sim.Time = 140;
sim_params.Sim = Sim;

Initial.Conditions.theta0 = deg2rad(90.01);         % Initial pitch [deg]
Initial.Conditions.tiltAngle = deg2rad(74.5);    % Initial pitch [deg]
Initial.Conditions.pitchRate = (0.0);           % Pitch rate [deg/s]

% --- Conditional Initial Conditions for Unit Testing ---
% Check if unit_test flag exists in the base workspace
unit_test_exists = evalin('base', 'exist(''unit_test'', ''var'')');
if unit_test_exists
    unit_test = evalin('base', 'unit_test');
else
    unit_test = false;
end

if unit_test
    fprintf('UNIT TEST MODE: Setting initial conditions from linearization script.\n');
    % This assumes the LQR design script has been run and 'params' exists in the base workspace
    try
        params = evalin('base', 'params');
        Initial.Conditions.V0 = params.V;
        Initial.Conditions.h0 = -params.Alt;
        fprintf('  - Initial Velocity (V0): %.2f m/s\n', Initial.Conditions.V0);
        fprintf('  - Initial Altitude (h0): %.2f m\n', Initial.Conditions.h0);
    catch ME
        warning('Could not find "params" struct in base workspace for unit test. Defaulting to 0.');
        Initial.Conditions.V0 = 0.1;
        Initial.Conditions.h0 = 0;
    end
else
    fprintf('STANDARD MODE: Using default initial conditions.\n');
    Initial.Conditions.V0 = 0.1;                     % Initial velocity [m/s]
    Initial.Conditions.h0 = 0;                       % Initial altitude [m] (positive up)
end
% --- End of Section ---

sim_params.Initial = Initial;

Actuators.Engine.BurnTime = readmatrix(heavySteveDataFilePath, 'Sheet', "Derived Properties", 'Range', "S4:S4");
Actuators.Nozzle.MaxDeflection = deg2rad(8);     % maxdef_nozzle [rad]
Actuators.Nozzle.RateLimit = deg2rad(150);       % rate_lim_nozzle [rad/s]
Actuators.Engine.MaxThrust = (27.6*10^3)*1;      % max_thrust [N]
sim_params.Actuators = Actuators;

Wind.shear = 20;                                % Wind shear [m/s]
sim_params.Wind = Wind;


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
Ref_Area = RocketAeroPhysical.Reference_Area;  % Create local variable for saving
fprintf('\nSaving simulation setup data to: %s\n', rocketSimDataFile);

% --- CORRECTED LINE ---
% Use the local variable 'Ref_Area' instead of the struct field 'sim_params.Ref_Area'
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
