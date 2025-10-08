% run_simulation.m - Main script to run the STEVE rocket simulation

% --- Start with a clean environment ---
clear;      % Clear workspace variables
clc;        % Clear command window
close all;  % Close all open figure windows

% Add script subdirectories to MATLAB path
% From run_simulation.m location, go to parent (scripts/), then to subdirectories
addpath(fullfile(fileparts(mfilename('fullpath')), 'analysis'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'setup'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

% Change to repo root directory (3 levels up from scripts/)
cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));

fprintf('--- Starting New Simulation Run ---\n');

% Initialize parameters
disp('Initializing simulation parameters...');
sim_params = initialize_sim_parameters();
disp('Initialization of sim_params complete.');

discrete_controller_design


% Load Simulink model
modelName = 'STEVE_Simulation';
disp(['Loading Simulink model: ', modelName, '...']);
load_system(modelName);
disp('Simulink model loaded.');

% Set simulation time
simTime = 70;
disp(['Simulation stop time set to: ', num2str(simTime), ' seconds.']);

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
generate_blender_csv(simOut); % Uses default suffix

% --- Plot results ---
% disp('Calling plot_simulation_results...');
% plot_simulation_results(simOut); % Pass the original simOut
% disp('--- Simulation Run Complete ---');
