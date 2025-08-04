function sim_params = initialize_sim_parameters()
% initialize_sim_parameters - Initializes all parameters for the STEVE rocket simulation.
% Self-contained, uses original variable names, allows a nose mass to be added.

fprintf('--- Initializing Simulation Parameters (Self-Contained, with Optional Nose Mass) ---\n');

% --- Project root
projectRoot = fileparts(fileparts(mfilename('fullpath')));
aeroMatFilePath = fullfile(projectRoot, 'Simulator_Core', 'input_data', 'CombinedAeroData_Grid_Symmetric_Corrected.mat');

% --- Core parameters
burn_time = 60;
timeStep = 0.005;
Sim.Timestep = 0.0001;

% --- Rocket physical parameters
RocketAeroPhysical.Diameter = 0.505;
RocketAeroPhysical.Length = 9.542;
RocketAeroPhysical.Reference_Area = 0.200296;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter;

% --- System Parameters
Actuators.Nozzle.MaxDeflection = deg2rad(8);
Actuators.Nozzle.RateLimit = deg2rad(150);
Actuators.Engine.MaxThrust = 27.6e3;
Actuators.Engine.BurnTime = burn_time;

Initial.Conditions.V0 = 0.01;
Initial.Conditions.h0 = 0;
Initial.Conditions.theta0 = deg2rad(90);
Initial.Conditions.pitchRate = 0.0;

Wind.shear = 0;

% --- Nose mass settings
add_nose_mass = true;
nose_mass = 115;      % [kg] mass to add
nose_mass_location = 0; % [m] from nose tip

% --- Time vector and # points
t_vec = (0:timeStep:burn_time)';
numPoints = numel(t_vec);

% =========================
% Generate Core Properties
% =========================
Time = t_vec;
Mass = -9.9996 * t_vec + 974.71;
COM_Z = 8.08517e-8 * t_vec.^4 - 5.18606e-6 * t_vec.^3 + 2.49473e-4 * t_vec.^2 ...
    - 8.67686e-3 * t_vec + 4.01782;
MOIx_Z = 1.103e-4 * t_vec.^2 - 2.226e-2 * t_vec + 1.915e4;
MOIy_Z = MOIx_Z;
MOIz_Z = -3.125e-1 * t_vec + 30.15;

if add_nose_mass
    fprintf('--- Adding %.2f kg nose mass at %.2f m from nose ---\n', nose_mass, nose_mass_location);
    L = RocketAeroPhysical.Length;
    CG_from_nose = L - COM_Z;
    total_mass = Mass + nose_mass;
    new_CG_from_nose = (Mass .* CG_from_nose + nose_mass * nose_mass_location) ./ total_mass;
    d_rocket = new_CG_from_nose - CG_from_nose;
    d_nose = new_CG_from_nose - nose_mass_location;
    new_MOIx_Z = MOIx_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_MOIy_Z = MOIy_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_MOIz_Z = MOIz_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_Mass = total_mass;
    new_COM_Z = L - new_CG_from_nose;
else
    new_Mass = Mass;
    new_COM_Z = COM_Z;
    new_MOIx_Z = MOIx_Z;
    new_MOIy_Z = MOIy_Z;
    new_MOIz_Z = MOIz_Z;
end

STeVeV1NoFins = table(Time, new_Mass, new_COM_Z, new_MOIx_Z, new_MOIy_Z, new_MOIz_Z, ...
    'VariableNames', {'Time', 'Mass', 'COM_Z', 'MOIx_Z', 'MOIy_Z', 'MOIz_Z'});

indices = 1:numPoints;
MassData.Time = single(STeVeV1NoFins.Time);
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z);
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z); % Roll
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);
avg_moi_lat = (MassData.MOI_Y + MassData.MOI_Z) / 2;
MassData.MOI_Y = avg_moi_lat;
MassData.MOI_Z = avg_moi_lat;
MassData.dMdt = gradient(MassData.Mass, timeStep);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStep);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStep);
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStep);
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
PrelookupData.TimeStep = timeStep;

% === Load Aero Data ===
fprintf('Loading aerodynamic data...\n');
if exist(aeroMatFilePath, 'file')
    load(aeroMatFilePath, 'CombinedAeroData');
    AeroData = CombinedAeroData;
    Mach_Breakpoints = AeroData.Breakpoints.Mach;
    Alpha_Breakpoints = AeroData.Breakpoints.Alpha;
    if isfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_')
        AeroData.Tables.Cnalpha = AeroData.Tables.Cnalpha_0_4deg__perRad_;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_');
    end
else
    error('Aerodynamic MAT file not found: %s', aeroMatFilePath);
end

% Placeholders for contour and thrust data
STeVe_Contour = table([0; RocketAeroPhysical.Length], [0; 0], {''; ''}, 'VariableNames', ["X", "Y", "Var3"]);
ThrustLookupData.Breakpoints.Pressure = single([0, 101325]);
ThrustLookupData.Tables.Thrust = single([27.6e3, 25.0e3]);
RocketAero.Physical = RocketAeroPhysical;
RocketAero.AeroData = AeroData;
Ref_Area = RocketAeroPhysical.Reference_Area;
sim_params.OutputDataPath = fullfile(projectRoot, 'Simulator_Core', 'output_data');
sim_params.Sim = Sim;
sim_params.Initial = Initial;
sim_params.Actuators = Actuators;
sim_params.Wind = Wind;
sim_params.RocketAero = RocketAero;
sim_params.PrelookupData = PrelookupData;
sim_params.ThrustLookupData = ThrustLookupData;
sim_params.Mach_Breakpoints = Mach_Breakpoints;
sim_params.Alpha_Breakpoints = Alpha_Breakpoints;
sim_params.Ref_Area = Ref_Area;

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
assignin('base', 'Ref_Area', Ref_Area);
assignin('base', 'STeVeV1NoFins', STeVeV1NoFins);
assignin('base', 'STeVe_Contour', STeVe_Contour);
assignin('base', 'MassData', MassData);
assignin('base', 'timeStep', timeStep);
assignin('base', 'inertia_tensor', inertia_tensor);
assignin('base', 'd_inertia_tensor', d_inertia_tensor);

% Print max mass and initial thrust-to-weight ratio
max_mass = new_Mass(1); % at t=0 (launch)
initial_TWR = Actuators.Engine.MaxThrust / (max_mass * 9.81);
fprintf('\n--- Max Launch Mass: %.2f kg', max_mass);
fprintf('\n--- Thrust-to-Weight Ratio at Launch: %.3f', initial_TWR);
fprintf('\n--- Min MOI: %.3f', min(new_MOIx_Z));
fprintf('\n--- Max CG: %.3f\n', max(new_COM_Z))

fprintf('\n--- Initialization Complete ---\n');
end
