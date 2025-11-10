% run_simulation.m - Main script to run the STEVE rocket simulation

% Ensure working directory is project root for correct relative paths
cd(fileparts(fileparts(mfilename('fullpath'))));
% Add all relevant paths for simulation
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/scripts');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/scripts/setup');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/scripts/analysis');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/scripts/utils');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/models');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/s_functions');
addpath('/Users/jonno/MATLAB-Drive/masters-rocket-control-repository/simulation_6dof/data');
% --- Start with a clean environment ---
clear;      % Clear workspace variables
clc;        % Clear command window
close all;  % Close all open figure windows

fprintf('--- Starting New Simulation Run ---\n');

% Initialize parameters
disp('Initializing simulation parameters...');
sim_params = initialize_sim_params_6dof();
disp('Initialization of sim_params complete.');

discrete_controller_design 


% Load Simulink model
modelName = 'STEVE_Simulation_6DOF';
disp(['Loading Simulink model: ', modelName, '...']);
load_system(modelName);
disp('Simulink model loaded.');

% Set simulation time
simTime = 80;
disp(['Simulation stop time set to: ', num2str(simTime), ' seconds.']);

euler_0 = deg2rad([0 80 0]);

% Run simulation
disp('Starting Simulink simulation...');
simOut = sim(modelName, 'StopTime', num2str(simTime));
disp('Simulink simulation finished.');

% --- Save simulation output to a .mat file ---
% disp('Saving simulation output to simOut.mat...');
% save('simOut.mat', 'simOut');
% disp('Save complete.');

% --- Generate CSV for Blender using the new function ---
% You can specify a custom suffix if desired, e.g., '_my_specific_run_60fps'
generate_blender_csv_6dof(simOut); % Uses default suffix
generate_ekf_csv;

% --- Plot results ---
% disp('Calling plot_simulation_results...');
% plot_simulation_results(simOut); % Pass the original simOut
% disp('--- Simulation Run Complete ---');
