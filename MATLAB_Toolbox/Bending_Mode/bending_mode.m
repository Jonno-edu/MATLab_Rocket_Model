%% Rocket Structural Design Script for Bending Stiffness
clear; clc;

% =========================================================================
% DEFINE ROCKET, LOAD, AND REQUIREMENTS
% =========================================================================

% Rocket Geometry & Material Properties
L = 9.542;          % Rocket Length (m)
outer_dia = 0.505;  % Rocket Outer Diameter (m)
E = 135e9;          % Young's Modulus for Carbon Fiber (Pa)
rho = 1600;         % Density for Carbon Fiber (kg/m^3)

% Maximum Load Case
max_thrust = 27.6e3;    % Max Thrust (N)
max_gimbal_deg = 5;     % Max gimbal angle (degrees)
F_side_max = max_thrust * sind(max_gimbal_deg);

% Performance Requirement
max_allowable_bend_deg = 0.3;
max_allowable_bend_rad = deg2rad(max_allowable_bend_deg);

% Key Locations
x_nozzle = L;
x_imu = 0.15 * L;
x_cp = 0.65 * L;

% Modal Constants
beta1 = 4.730 / L;
sigma1 = 0.9825;

fprintf('Design requirement: Max nozzle bend angle <= %.2f degrees\n', max_allowable_bend_deg);
fprintf('Under a max side force of %.0f N.\n\n', F_side_max);

% =========================================================================
% ITERATIVELY SOLVE FOR WALL THICKNESS
% =========================================================================

wall_thickness_m = 0.001;  % Start with 1mm
increment_m = 0.0001;      % Increment by 0.1mm

fprintf('Searching for required wall thickness...\n');

while true
    % Calculate geometric properties
    inner_dia = outer_dia - 2 * wall_thickness_m;
    I = (pi/64) * (outer_dia^4 - inner_dia^4);
    A = (pi/4) * (outer_dia^2 - inner_dia^2);
    mu = rho * A;
    
    % CORRECTED: Use beam theory for static deflection
    % For a cantilever beam with end load, tip slope = F*L^2 / (2*E*I)
    calculated_bend_rad = abs((F_side_max * L^2) / (2 * E * I));
    
    if calculated_bend_rad <= max_allowable_bend_rad
        fprintf('Success! Found a suitable thickness.\n\n');
        break;
    else
        wall_thickness_m = wall_thickness_m + increment_m;
    end
end

% =========================================================================
% EXPORT FINAL DESIGN PARAMETERS AND SIMULINK GAINS
% =========================================================================

final_thickness_mm = wall_thickness_m * 1000;

% Calculate natural frequency
omega_n_sq = (beta1^4 * E * I) / mu;
final_freq_hz = sqrt(omega_n_sq) / (2*pi);

fprintf('--- FINAL DESIGN --- \n');
fprintf('Required Wall Thickness: %.2f mm\n', final_thickness_mm);
fprintf('Resulting 1st Bending Frequency: %.2f Hz\n\n', final_freq_hz);

% Calculate Simulink gains using mode shapes
Phi_1 = @(x) (cosh(beta1*x) + cos(beta1*x)) - sigma1 * (sinh(beta1*x) + sin(beta1*x));
Phi_1_prime = @(x) beta1 * ((sinh(beta1*x) - sin(beta1*x)) - sigma1 * (cosh(beta1*x) + cos(beta1*x)));

gain_force_nozzle = Phi_1(x_nozzle);
gain_force_cp = Phi_1(x_cp);
gain_slope_nozzle = Phi_1_prime(x_nozzle);
gain_slope_imu = Phi_1_prime(x_imu);
gain_disp_nozzle = Phi_1(x_nozzle);

fprintf('--- GAINS FOR SIMULINK MODEL ---\n');
fprintf('Modal Force Participation (from Nozzle): %.4f\n', gain_force_nozzle);
fprintf('Modal Force Participation (from CP):     %.4f\n', gain_force_cp);
fprintf('Slope at Nozzle (for Angle):             %.4f\n', gain_slope_nozzle);
fprintf('Slope at IMU (for Angle):                %.4f\n', gain_slope_imu);
fprintf('Displacement at Nozzle (for Position):   %.4f\n', gain_disp_nozzle);

