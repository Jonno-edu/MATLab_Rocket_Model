%% Automated Linearization of a Nonlinear Simulink Model

clear; clc; close all;

%% 1. Configuration
% --- User-defined settings ---
% Path to your simulation output file
filename = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/output_data/simOut_heavy_benchmark.mat';

% Name of your Simulink model file (without the .slx extension)
modelName = 'STEVE_Simulation'; % Updated with your model name

% Time point for linearization (in seconds)
linearizationTime = 15.0; 

%% 2. Load Simulation Data
if exist(filename, 'file')
    load(filename);
    disp('Successfully loaded flight data.');
else
    error('Data file not found. Please check the path: %s', filename);
end

% Check if the expected simOut variable exists after loading
if ~exist('simOut', 'var')
    error('The loaded MAT-file does not contain a "simOut" variable.');
end

%% 3. Define the Operating Point for Linearization
% Find the index corresponding to the linearization time
time_index = find(simOut.tout >= linearizationTime, 1);

if isempty(time_index)
    error('Linearization time is beyond the simulation time range.');
end

fprintf('Extracting operating point at t = %.2f seconds.\n', simOut.tout(time_index));

% Extract the state values at the operating point
% IMPORTANT: The paths and variable names must match your simulation logs.
try
    alpha_op = simOut.States.alpha.Data(time_index); % Angle of attack (rad)
    q_op     = simOut.States.q.Data(time_index);     % Pitch rate (rad/s)
    theta_op = simOut.States.theta.Data(time_index); % Pitch angle (rad)
    
    % If your model includes actuator states, add them here
    % act_pos_op = simOut.States.act_pos.Data(time_index);
    % act_rate_op = simOut.States.act_rate.Data(time_index);

    % Extract the input value at the operating point
    delta_T_op = simOut.Inputs.delta_T.Data(time_index); % TVC angle (rad)
catch ME
    disp('Error extracting data from the simOut structure.');
    disp('Please check that the variable paths match your simulation output.');
    rethrow(ME);
end

%% 4. Perform Automated Linearization
% Check for the required toolbox
if ~license('test', 'Simulink_Control_Design')
    error('Simulink Control Design toolbox is required for linearization.');
end

% Specify the input and output points for the linearization in your model
% This assumes you have defined these points in your Simulink model properties
io = getlinio(modelName);

% Create an operating point specification from the model
op = operpoint(modelName);

% Set the known state and input values at the operating point
% The indices below (e.g., op.States(1)) might need to be adjusted
% depending on how Simulink orders your states. You can inspect 'op'
% in the workspace to confirm the correct order.
op.States(1).x = alpha_op;
op.States(2).x = q_op;
op.States(3).x = theta_op;
op.Inputs(1).u = delta_T_op;

% Perform the linearization
fprintf('Linearizing Simulink model "%s"...\n', modelName);
sys_auto = linearize(modelName, io, op);

%% 5. Display the Results
% The output 'sys_auto' is a state-space object containing the A, B, C, D matrices.
A_auto = sys_auto.A;
B_auto = sys_auto.B;

fprintf('\n--- Automated Linearization Results ---\n');
disp('Automatically Linearized A Matrix:');
disp(A_auto);

disp('Automatically Linearized B Matrix:');
disp(B_auto);
fprintf('--------------------------------------\n');
