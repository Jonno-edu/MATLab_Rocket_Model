clear; clc; close all;

% --- Parameters ---
burn_time = 60;
timeStep = 0.1;
nose_mass_values = linspace(0, 200, 21); % [kg] mass to add
nose_mass_location = 0; % [m] from nose tip
RocketAeroPhysical.Length = 15.0; % [m] Set rocket length

% --- Time vector ---
t_vec = (0:timeStep:burn_time)';
numPoints = numel(t_vec);

% =========================
% Generate Core Properties
% =========================
Mass = -9.9996 * t_vec + 974.71;
COM_Z = 8.08517e-8 * t_vec.^4 - 5.18606e-6 * t_vec.^3 + 2.49473e-4 * t_vec.^2 ...
    - 8.67686e-3 * t_vec + 4.01782;
MOIx_Z = 1.103e-4 * t_vec.^2 - 2.226e-2 * t_vec + 1.915e4;
MOIy_Z = MOIx_Z;
MOIz_Z = -3.125e-1 * t_vec + 30.15;

% =========================
% Plotting Setup
% =========================
figure('Name', 'Bode Magnitude Plots');
hold on; % Keep the plot active to overlay all bode plots

% =========================
% Main Loop
% =========================
for i = 1:numel(nose_mass_values)
    nose_mass = nose_mass_values(i);
    
    % This print statement is helpful for tracking progress
    fprintf('Calculating for nose mass: %.2f kg\n', nose_mass);
    
    L = RocketAeroPhysical.Length;
    CG_from_nose = L - COM_Z;
    total_mass = Mass + nose_mass;
    
    new_CG_from_nose = (Mass .* CG_from_nose + nose_mass * nose_mass_location) ./ total_mass;
    
    d_rocket = new_CG_from_nose - CG_from_nose;
    d_nose = new_CG_from_nose - nose_mass_location;
    
    new_MOIy_Z = MOIy_Z + Mass .* d_rocket.^2 + nose_mass * d_nose.^2;

    % --- Plant Parameters (using properties at t=0) ---
    T = 27607;        % Thrust (N)
    l_CG = L - new_CG_from_nose(1); % Nozzle to CG distance at t=0
    I_y = new_MOIy_Z(1);      % Pitch moment of inertia at t=0
    
    % --- Plant: Pitch Rate ---
    k_plant = T * l_CG / I_y;
    pitch_rate_plant = tf([k_plant], [1 0]);
    
    % --- Actuator Model ---
    omega_n = 62;     % Actuator natural frequency (rad/s)
    zeta = 0.505;     % Actuator damping ratio
    actuator = tf([omega_n^2], [1 2*zeta*omega_n omega_n^2]);
    
    % --- Combined Plant ---
    plant_inner = series(actuator, pitch_rate_plant);
    
    % --- Plot Bode Magnitude on the current figure ---
    w_range = logspace(-1, 3, 1000); % Define a consistent frequency range
    [mag, ~, wout] = bode(plant_inner, w_range);
    mag_db = 20 * log10(squeeze(mag));
    
    % Plot the magnitude data and set a display name for the legend
    bode(plant_inner)

end

% =========================
% Finalize the Plot
% =========================
hold off; % Release the plot
grid on;
title('Bode Magnitude Plots for Different Nose Masses');
