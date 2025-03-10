%% initialize.m - Use Index-Based Lookup for Prelookup
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
Actuators.Engine.DelayBeforeStart = 5;           %Delay time in seconds

%% Import Data (Verify Time Spacing First)
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

%% Create Index-Based Prelookup Data
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
MassData.MOI_Z = single(STeVeV1NoFins.MOIy_Z);   % Y→Z

%% Enforce Axisymmetry and Calculate Derivatives
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

%% Export to Simulink
assignin('base', 'PrelookupData', PrelookupData);
assignin('base', 'Initial', Initial);
assignin('base', 'Actuators', Actuators);

%% Final Validation Message
fprintf('\n--- Index-Based Prelookup Setup Complete ---\n');
disp('Exported Prelookup Data to Simulink workspace.');
disp('To use in Simulink, convert simulation time to index using:');
fprintf('  index = (t - %.4f)/%.4f + 1\n', STeVeV1NoFins.Time(1), timeStep);
