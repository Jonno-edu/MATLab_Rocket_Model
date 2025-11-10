%% Rocket Structural Parameter Setup Script
% This script calculates the required wall thickness and defines all the
% constant parameters needed for the Simulink flexible body model.
%
% RUN THIS SCRIPT ONCE BEFORE STARTING THE SIMULATION.

fprintf('--- Running Vehicle Structural Setup ---\n\n');

% =========================================================================
% 1. DEFINE CONSTANTS & REQUIREMENTS
% =========================================================================
% --- Rocket Geometry & Material Properties ---
L = 9.542;          % Rocket Length (m)
outer_dia = 0.505;  % Rocket Outer Diameter (m)
E = 135e9;          % Young's Modulus for Carbon Fiber (Pa)
rho = 1600;         % Density for Carbon Fiber (kg/m^3)
zeta = 0.015;       % Structural Damping Ratio (1.5%)

% --- Load Case & Bending Requirement ---
F_side_max = 2405;  % Max side force (N) from previous runs
max_allowable_bend_deg = 0.30;
max_allowable_bend_rad = deg2rad(max_allowable_bend_deg);

% --- Key Locations & Modal Constants ---
x_nozzle = L;
beta1 = 4.730 / L;
sigma1 = 0.9825;

% =========================================================================
% 2. SOLVE FOR WALL THICKNESS (Same logic as before)
% =========================================================================
wall_thickness_m = 0.001;
increment_m = 0.0001;
while true
    inner_dia = outer_dia - 2 * wall_thickness_m;
    I = (pi/64) * (outer_dia^4 - inner_dia^4);
    calculated_bend_rad = abs((F_side_max * L^2) / (2 * E * I));
    if calculated_bend_rad <= max_allowable_bend_rad, break;
    else, wall_thickness_m = wall_thickness_m + increment_m; end
end

% =========================================================================
% 3. CREATE FINAL SIMULINK PARAMETERS IN WORKSPACE
% =========================================================================
% Calculate the final properties based on the solved thickness
A = (pi/4) * (outer_dia^2 - inner_dia^2);
mu = rho * A;
omega_n_sq = (beta1^4 * E * I) / mu;
omega_n = sqrt(omega_n_sq);

% Define the mode shape function handle
Phi_1 = @(x) (cosh(beta1*x) + cos(beta1*x)) - sigma1 * (sinh(beta1*x) + sin(beta1*x));

% Create a clean structure 'flex_params' to hold these values
flex_params.omega_n = omega_n;
flex_params.omega_n_sq = omega_n_sq;
flex_params.zeta = zeta;
flex_params.x_nozzle = x_nozzle;
flex_params.Phi_1 = Phi_1;

% Also create the gain parameters for convenience
gains.slope_at_imu = Phi_1(0.15 * L);
gains.slope_at_nozzle = Phi_1(x_nozzle);


fprintf('--- FINAL DESIGN --- \n');
fprintf('Required Wall Thickness: %.2f mm\n', wall_thickness_m * 1000);
fprintf('Resulting 1st Bending Frequency: %.2f Hz\n\n', omega_n / (2*pi));
disp('Setup complete. The ''flex_params'' struct is now in the workspace.');

