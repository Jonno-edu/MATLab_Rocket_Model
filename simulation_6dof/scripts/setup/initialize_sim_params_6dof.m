function sim_params = initialize_sim_params_6dof()
% initialize_sim_params_6dof - Initializes all parameters for the STEVE rocket simulation (6DOF).
% Self-contained, uses original variable names, allows a nose mass to be added,
% and now includes flexible body parameter initialization.

fprintf('--- Initializing Simulation Parameters (6DOF, Self-Contained, with Optional Nose Mass) ---\n');

% --- Project root (robust relative path)
thisFile = mfilename('fullpath');
scriptDir = fileparts(thisFile);
repoRoot = fileparts(fileparts(fileparts(scriptDir))); % Go up to repo root
aeroMatFilePath = fullfile(repoRoot, 'simulation_6dof', 'data', 'input', 'CombinedAeroData_6DOF.mat');

% --- Core parameters
burn_time = 60;
warm_up_time = 10;
timeStep = 0.005;
Sim.Timestep = 0.0001;

% --- Rocket physical parameters
RocketAeroPhysical.Diameter = 0.505;
RocketAeroPhysical.Length = 9.542;
RocketAeroPhysical.Reference_Area = 0.200296;
RocketAeroPhysical.Reference_Length = RocketAeroPhysical.Diameter;

% --- Sensor Parameters
Sensors.imu_dt = 1/1000;
Sensors.gps_dt = 1/40;
Sensors.accel_max_m_s_2 = 392.4000;
Sensors.a_rand_walk = 2.5000e-04;
Sensors.a_bias_inst = 1.4715e-04;
Sensors.g_dyn_range_rad_s = 34.9066;
Sensors.g_rand_walk = 4.3633e-05;
Sensors.g_bias_inst_rad_s = 1.9877e-05;
Sensors.bias_from_accel = 3.4600e-06;

% --- System Parameters
Actuators.Nozzle.MaxDeflection = deg2rad(8);
Actuators.Nozzle.RateLimit = deg2rad(150);
Actuators.Engine.MaxThrust = 27.6e3;
Actuators.Engine.BurnTime = burn_time;

Initial.Conditions.V0 = 0.01;
Initial.Conditions.h0 = 0;
Initial.Conditions.theta0 = deg2rad(80);
Initial.Conditions.pitchRate = 0.0;

Wind.shear = 0;
launchpad_alt = 100; % [m]

% --- Nose mass settings
add_nose_mass = true;
nose_mass = 0;         % [kg] mass to add
nose_mass_location = 0;  % [m] from nose tip

% --- Time vector and # points
t_vec = (0:timeStep:burn_time)';
numPoints = numel(t_vec);

% ==========================================================
% NEW: Initialize Bending Mode Parameters
% ==========================================================
fprintf('--- Initializing Bending Mode Parameters ---\n');

% --- Static Structural Design ---
% Based on our previous analysis, we use a fixed wall thickness
flex.wall_thickness_mm = 3.20; 

% --- Material & Damping Properties ---
flex.E_modulus = 135e9;  % Young's Modulus for Carbon Fiber (Pa)
flex.density = 1600;    % Density for Carbon Fiber (kg/m^3)
flex.zeta = 0.015;      % Structural Damping Ratio (1.5%)

% --- Calculations ---
wall_thickness_m = flex.wall_thickness_mm / 1000;
L = RocketAeroPhysical.Length;
outer_dia = RocketAeroPhysical.Diameter;

inner_dia = outer_dia - 2 * wall_thickness_m;
I = (pi/64) * (outer_dia^4 - inner_dia^4); % Area Moment of Inertia
A = (pi/4) * (outer_dia^2 - inner_dia^2);  % Cross-sectional area
mu = flex.density * A; % Mass per unit length

% --- Modal Constants (for Free-Free Beam) ---
beta1 = 4.730 / L;
sigma1 = 0.9825;

% --- Final Flexible Body Parameters for Simulink ---
omega_n_sq = (beta1^4 * flex.E_modulus * I) / mu;
omega_n = sqrt(omega_n_sq);

% --- Create the 'flex_params' struct for the Simulink function block ---
% NOTE: We pass the RAW constants, not a function handle.
flex_params.omega_n = omega_n;
flex_params.omega_n_sq = omega_n_sq;
flex_params.zeta = flex.zeta;
flex_params.x_nozzle = L;
flex_params.beta1 = beta1;
flex_params.sigma1 = sigma1;

% The derivative of the mode shape function
Phi_1_prime = @(x) beta1 * ((sinh(beta1*x) - sin(beta1*x)) - sigma1 * (cosh(beta1*x) + cos(beta1*x)));

% Calculate the gain to convert q1 to nozzle angle in radians
flex_params.gain_nozzle_angle = Phi_1_prime(L); % where L is the rocket length

fprintf('Gain to convert q1 to nozzle angle (rad): %.4f\n', flex_params.gain_nozzle_angle);


fprintf('Using a fixed wall thickness of %.2f mm.\n', flex.wall_thickness_mm);
fprintf('Resulting 1st Bending Frequency: %.2f Hz\n', omega_n / (2*pi));


% =========================
% Generate Core Properties
% =========================
Time = t_vec;
% Mass, CG, MOI polynomials
Mass  = -9.9996 * t_vec + 974.71;
COM_Z = 8.08517e-8 * t_vec.^4 - 5.18606e-6 * t_vec.^3 + 2.49473e-4 * t_vec.^2 ...
      - 8.67686e-3 * t_vec + 4.01782;
MOIx_Z = 1.103e-4 * t_vec.^2 - 2.226e-2 * t_vec + 1.915e4;
MOIy_Z = MOIx_Z;
MOIz_Z = -3.125e-1 * t_vec + 30.15;

% --- Optional nose mass addition
if add_nose_mass
    fprintf('--- Adding %.2f kg nose mass at %.2f m from nose ---\n', nose_mass, nose_mass_location);
    L = RocketAeroPhysical.Length;
    CG_from_nose = L - COM_Z;
    total_mass = Mass + nose_mass;
    new_CG_from_nose = (Mass .* CG_from_nose + nose_mass * nose_mass_location) ./ total_mass;
    d_rocket = new_CG_from_nose - CG_from_nose;
    d_nose   = new_CG_from_nose - nose_mass_location;
    new_MOIx_Z = MOIx_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_MOIy_Z = MOIx_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_MOIz_Z = MOIz_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;
    new_Mass  = total_mass;
    new_COM_Z = L - new_CG_from_nose;
else
    new_Mass  = Mass;
    new_COM_Z = COM_Z;
    new_MOIx_Z = MOIx_Z;
    new_MOIy_Z = MOIy_Z;
    new_MOIz_Z = MOIz_Z;
end

% Assemble table
STeVeV1NoFins = table(Time, new_Mass, new_COM_Z, new_MOIx_Z, new_MOIy_Z, new_MOIz_Z, ...
    'VariableNames', {'Time', 'Mass', 'COM_Z', 'MOIx_Z', 'MOIy_Z', 'MOIz_Z'});

% Prelookup arrays
indices = 1:numPoints;
MassData.Time = single(STeVeV1NoFins.Time);
MassData.Mass = single(STeVeV1NoFins.Mass);
MassData.COM_X = single(STeVeV1NoFins.COM_Z);
% Axis mapping: MOI_X=roll, MOI_Y=pitch, MOI_Z=yaw
MassData.MOI_X = single(STeVeV1NoFins.MOIz_Z); % Roll
MassData.MOI_Y = single(STeVeV1NoFins.MOIx_Z);
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);
% Average lateral MOI for symmetry
avg_moi_lat = (MassData.MOI_Y + MassData.MOI_Z) / 2;
MassData.MOI_Y = avg_moi_lat;
MassData.MOI_Z = avg_moi_lat;

% Time derivatives
MassData.dMdt   = gradient(MassData.Mass,  timeStep);
MassData.dIdt_X = gradient(MassData.MOI_X, timeStep);
MassData.dIdt_Y = gradient(MassData.MOI_Y, timeStep);
MassData.dIdt_Z = gradient(MassData.MOI_Z, timeStep);

% Inertia tensors over time
inertia_tensor   = zeros(3, 3, numPoints, 'single');
d_inertia_tensor = zeros(3, 3, numPoints, 'single');
for i = 1:numPoints
    inertia_tensor(:,:,i)   = diag([MassData.MOI_X(i), MassData.MOI_Y(i), MassData.MOI_Z(i)]);
    d_inertia_tensor(:,:,i) = diag([MassData.dIdt_X(i), MassData.dIdt_Y(i), MassData.dIdt_Z(i)]);
end

% Prelookup package
PrelookupData.Breakpoints.Index = single(indices);
PrelookupData.Tables.Mass       = MassData.Mass;
PrelookupData.Tables.COM_X      = MassData.COM_X;
PrelookupData.Tables.MOI_X      = MassData.MOI_X;
PrelookupData.Tables.MOI_Y      = MassData.MOI_Y;
PrelookupData.Tables.MOI_Z      = MassData.MOI_Z;
PrelookupData.Tables.dMdt       = MassData.dMdt;
PrelookupData.Tables.dIdt_X     = MassData.dIdt_X;
PrelookupData.Tables.dIdt_Y     = MassData.dIdt_Y;
PrelookupData.Tables.dIdt_Z     = MassData.dIdt_Z;
PrelookupData.Tables.InertialTensor   = inertia_tensor;
PrelookupData.Tables.dInertialTensor  = d_inertia_tensor;
PrelookupData.TimeStep = timeStep;

% === Load Aero Data ===
fprintf('Loading aerodynamic data...\n');
if exist(aeroMatFilePath, 'file')
    load(aeroMatFilePath, 'CombinedAeroData');
    AeroData = CombinedAeroData;
    % Breakpoints
    Mach_Breakpoints  = AeroData.Breakpoints.Mach;
    Alpha_Breakpoints = AeroData.Breakpoints.Alpha;

    % ... (rest of your aero data processing remains unchanged) ...
    if isfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_')
        AeroData.Tables.Cnalpha = AeroData.Tables.Cnalpha_0_4deg__perRad_;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha_0_4deg__perRad_');
    end
    if isfield(AeroData.Tables, 'CAPower_On')
        AeroData.Tables.CA = AeroData.Tables.CAPower_On;
        AeroData.Tables = rmfield(AeroData.Tables, 'CAPower_On');
    end
    if isfield(AeroData.Tables, 'Cnalpha')
        AeroData.Tables.CNalpha = AeroData.Tables.Cnalpha;
        AeroData.Tables = rmfield(AeroData.Tables, 'Cnalpha');
    end
    if isfield(AeroData.Tables, 'CN')
        AeroData.Tables.CY = AeroData.Tables.CN;
    end
    if isfield(AeroData.Tables, 'CNalpha')
        AeroData.Tables.CYbeta = AeroData.Tables.CNalpha;
    end
    if ~isfield(AeroData.Breakpoints, 'Beta')
        AeroData.Breakpoints.Beta = Alpha_Breakpoints;
    end
    Beta_Breakpoints = AeroData.Breakpoints.Beta;
    if isfield(AeroData.Breakpoints, 'AlphaTot')
        AlphaTot_Breakpoints = AeroData.Breakpoints.AlphaTot;
    else
        AlphaTot_Breakpoints = Alpha_Breakpoints;
        AeroData.Breakpoints.AlphaTot = AlphaTot_Breakpoints;
    end
    if ~isfield(AeroData.Tables, 'CA_AlphaTot') && isfield(AeroData.Tables, 'CA'), AeroData.Tables.CA_AlphaTot = AeroData.Tables.CA; end
    if ~isfield(AeroData.Tables, 'CP_AlphaTot') && isfield(AeroData.Tables, 'CP'), AeroData.Tables.CP_AlphaTot = AeroData.Tables.CP; end
    if ~isfield(AeroData.Tables,'Cmq'), AeroData.Tables.Cmq = zeros(numel(Mach_Breakpoints), numel(Alpha_Breakpoints), 'like', AeroData.Tables.CN); end
    if ~isfield(AeroData.Tables,'Cmq_beta'), AeroData.Tables.Cmq_beta = AeroData.Tables.Cmq; end
    AeroData.Metadata.Axisymmetry.CY_from_CN = true;
    AeroData.Metadata.Axisymmetry.CYbeta_from_CNa = isfield(AeroData.Tables,'CYbeta');

else
    error('Aerodynamic MAT file not found: %s', aeroMatFilePath);
end

% Placeholders for contour and thrust data
STeVe_Contour = table([0; RocketAeroPhysical.Length], [0; 0], {''; ''}, 'VariableNames', ["X", "Y", "Var3"]);
ThrustLookupData.Breakpoints.Pressure = single([0, 101325]);
ThrustLookupData.Tables.Thrust = single([27.6e3, 25.0e3]);

% Pack outputs
RocketAero.Physical = RocketAeroPhysical;
RocketAero.AeroData = AeroData;
Ref_Area = RocketAeroPhysical.Reference_Area;

sim_params.OutputDataPath     = fullfile(repoRoot, 'simulation_6dof', 'data', 'output');
sim_params.Sim                = Sim;
sim_params.Initial            = Initial;
sim_params.Actuators          = Actuators;
sim_params.Wind               = Wind;
sim_params.RocketAero         = RocketAero;
sim_params.PrelookupData      = PrelookupData;
sim_params.ThrustLookupData   = ThrustLookupData;
sim_params.Mach_Breakpoints   = Mach_Breakpoints;
sim_params.Alpha_Breakpoints  = Alpha_Breakpoints;
sim_params.Beta_Breakpoints   = Beta_Breakpoints;
sim_params.AlphaTot_Breakpoints = AlphaTot_Breakpoints;
sim_params.Ref_Area           = Ref_Area;
sim_params.FlexBody           = flex_params; % <-- ADDED

% Push to base workspace (Simulink convenience)
assignin('base', 'Sim', Sim);
assignin('base', 'Initial', Initial);
assignin('base', 'Actuators', Actuators);
assignin('base', 'launchpad_alt', launchpad_alt);
assignin('base', 'warm_up_time', warm_up_time);
assignin('base', 'Wind', Wind);
assignin('base', 'RocketAeroPhysical', RocketAeroPhysical);
assignin('base', 'rocket_length', RocketAeroPhysical.Length);
assignin('base', 'AeroData', AeroData);
assignin('base', 'Mach_Breakpoints', Mach_Breakpoints);
assignin('base', 'Alpha_Breakpoints', Alpha_Breakpoints);
assignin('base', 'Beta_Breakpoints', Beta_Breakpoints);
assignin('base', 'AlphaTot_Breakpoints', AlphaTot_Breakpoints);
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
assignin('base', "Sensors", Sensors);
assignin('base', "flex_params", flex_params);

% Print summary
max_mass = new_Mass(1); % at t=0 (launch)
initial_TWR = Actuators.Engine.MaxThrust / (max_mass * 9.81);
fprintf('\n--- Max Launch Mass: %.2f kg', max_mass);
fprintf('\n--- Thrust-to-Weight Ratio at Launch: %.3f', initial_TWR);
fprintf('\n--- Min MOI: %.3f', min(new_MOIx_Z));
fprintf('\n--- Max CG: %.3f', max(new_COM_Z));
fprintf('\n--- 1st Bending Mode Frequency: %.2f Hz', flex_params.omega_n / (2*pi)); % <-- ADDED
if isfield(AeroData.Tables,'CY'), fprintf('\n--- Lateral CY table ready (axisymmetric mirror of CN).'); end
if isfield(AeroData.Tables,'CYbeta'), fprintf('\n--- Lateral CYbeta table ready (mirror of CNalpha).'); end
if isfield(AeroData.Tables,'Cmq'), fprintf('\n--- Cmq ready (lumped pitch-rate damping).'); end
if isfield(AeroData.Tables,'Cmq_beta'), fprintf('\n--- Cmq_beta ready (mirrored beta-plane damping).'); end
if isfield(AeroData.Tables,'CA_AlphaTot'), fprintf('\n--- CA_AlphaTot present (use with total AoA).'); end
if isfield(AeroData.Tables,'CP_AlphaTot'), fprintf('\n--- CP_AlphaTot present (use with total AoA).'); end
fprintf('\n--- Initialization Complete ---\n');

end
