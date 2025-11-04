% build_s_function.m
% This script compiles the S-Function wrapper and links it with the
% shared controller source code.

% Define paths relative to this script's location
sfun_folder = '../s_functions';
shared_src_folder = '../../src/controller';

% Construct the source file paths
wrapper_file = fullfile(sfun_folder, 'sfcontroller_wrapper.c');
controller_file = fullfile(shared_src_folder, 'controller.c');

% Construct the include path argument
include_path = ['-I' shared_src_folder];

% Run the mex command with verbose output
fprintf('Compiling S-function...\n');

try
    mex('-v', ...
        wrapper_file, ...
        controller_file, ...
        include_path, ...
        '-outdir', pwd);

    fprintf('✓ Successfully compiled sfcontroller_wrapper.%s\n', mexext);

catch ME
    fprintf('✗ Compilation failed:\n%s\n', ME.message);
end
