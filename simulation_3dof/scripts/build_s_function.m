% build_s_function.m
% This script compiles the S-Function wrapper and links it with the
% shared controller source code.

% Get absolute paths based on script location
thisFile = mfilename('fullpath');
scriptDir = fileparts(thisFile);
sim3dofRoot = fileparts(scriptDir); % Go up from scripts/ to simulation_3dof/
repoRoot = fileparts(sim3dofRoot); % Go up from simulation_3dof/ to repo root

% Define paths
sfun_folder = fullfile(sim3dofRoot, 's_functions');
shared_src_folder = fullfile(repoRoot, 'src', 'controller');

% Construct the source file paths
wrapper_file = fullfile(sfun_folder, 'sfcontroller_wrapper.c');
controller_file = fullfile(shared_src_folder, 'controller.c');

% Construct the include path argument
include_path = ['-I' shared_src_folder];

% Run the mex command
fprintf('Building S-Function...\n');
mex(include_path, wrapper_file, controller_file);
fprintf('Build complete.\n');
