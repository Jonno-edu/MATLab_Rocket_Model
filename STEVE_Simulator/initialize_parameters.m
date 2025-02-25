%% Nozzle Actuators
wn_act = 1; % rad/s (update for actuators)
z_act = 0.3;
maxdef_nozzle = (30) * pi/180; % rad
mindef_nozzle = -maxdef_nozzle;
rate_lim_nozzle = (1000) * pi/180; % rad

nozzle_moment_arm = 0.01;

max_thrust = 500; % N

%% Initialize Aerodynamic Coefficient Data
% Load the dataset
data = readtable('aeroData.csv');

% Extract unique Mach and Alpha values
machValues  = unique(data.Mach);
alphaValues = unique(data.Alpha);

% Initialize grids for CD, CL, CN, etc.
%   Rows:    alpha
%   Columns: mach
CD_grid = nan(length(alphaValues), length(machValues));
CL_grid = nan(length(alphaValues), length(machValues));
CN_grid = nan(length(alphaValues), length(machValues));

% Tolerance for matching floating-point values
tol = 1e-8;  % Adjust as needed

% Fill the grids with corresponding values
for i = 1:height(data)
    alphaVal = data.Alpha(i);
    machVal  = data.Mach(i);

    % Find the row index for alphaVal (within tolerance)
    rowCandidates = find(abs(alphaValues - alphaVal) < tol);
    % Find the column index for machVal (within tolerance)
    colCandidates = find(abs(machValues  - machVal ) < tol);

    if isempty(rowCandidates) || isempty(colCandidates)
        % If no exact match is found, skip or warn
        fprintf('No alpha/mach match for (%.6f, %.6f)\n', alphaVal, machVal);
    else
        % Use the first match if multiple found
        row = rowCandidates(1);
        col = colCandidates(1);

        % Assign the values
        CD_grid(row, col) = data.CD(i);
        CL_grid(row, col) = data.CL(i);
        CN_grid(row, col) = data.CN(i);
    end
end

CD_grid;

% Save the variables for Simulink
save('lookup_data.mat', 'machValues', 'alphaValues', ...
                        'CD_grid', 'CL_grid', 'CN_grid');
