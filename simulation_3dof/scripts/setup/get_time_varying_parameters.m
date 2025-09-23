function Lookup = get_time_varying_parameters(burn_time)
% get_time_varying_parameters - Returns mass, CG, and MOI over burn time.
%
% Inputs:
%   burn_time: Total engine burn duration in seconds.
%
% Outputs:
%   Lookup: A struct containing time series vectors for Mass, CG, MOIx, and MOIz.

fprintf('--- Generating time-varying parameters from polynomials ---\n');

% --- Simulation Time Setup ---
dt = 0.005; % Time step for the lookup table
t = (0:dt:burn_time)'; % Time vector

% --- Polynomial Equations from Fits ---
Mass = -9.99960 * t + 974.709;
MOIx = 1.103e-4 * t.^2 - 2.226e-2 * t + 1.915e4;
CG = 8.08517e-8 * t.^4 - 5.18606e-6 * t.^3 + 2.49473e-4 * t.^2 ...
    - 8.67686e-3 * t + 4.01782;
MOIz = -3.125e-1 * t + 30.15;

% --- Package data into the output struct ---
Lookup.Time = t;
Lookup.Mass = Mass;
Lookup.CG = CG;
Lookup.MOIx = MOIx; % Corresponds to pitch/yaw axis
Lookup.MOIz = MOIz; % Corresponds to roll axis

end
