clear
clc
close all


%% 1. Define Fixed System Parameters
% These parameters do not change significantly with time.
S = 0.200296;           % Reference area (m²)
L = 9.542;              % Reference length (m)
L_arm = 3.9;            % TVC moment arm from CG to nozzle (m)
% Aerodynamic coefficients
CLa = 2.0;
Cma = 0.885; % Unstable
Cmq = -1.05;

%% 2. Define Time-Varying Flight Data
time_vector =      [0,     35,     65,     90,     125,    145];
velocity_vector =  [1.0,   150.6,  387.6,  728.5,  1604.7, 2523.6];
density_vector =   [1.225, 0.975,  0.4,    0.05,   0.0135, 0.005];
mass_vector =      [1873,  1528.1, 1228.4, 977.8,  628.3,  415.1];
thrust_vector =    [2.384e4, 2.48e4, 2.67e4, 2.75e4, 2.76e4, 2.76e4];
inertia_vector =   [3.31e4, 2.77e4, 2.31e4, 1.93e4, 1.36e4, 1.05e4];

%% 3. Define LQR Tuning for the Airframe Controller (ROBUST BASELINE)
% State vector is [alpha, theta, theta_dot]
max_alpha     = 3*pi/180;
max_theta     = 4*pi/180;
max_theta_dot = 0.5*pi/180;
Q_diag = [1/max_alpha^2, 1/max_theta^2, 1/max_theta_dot^2];
Q = diag(Q_diag);

% --- NEW ROBUST TUNING: Use a large, constant R ---
% We are making the control cost extremely high to force the LQR to find
% a low-gain, robust solution. This is our safe starting point.
R = 10000; % A large penalty on TVC command

%% 4. Main Loop: Design Airframe Controller and Analyze Stability
num_points = length(time_vector);
K_gains = zeros(num_points, 3);
stability_results = zeros(num_points, 3);

fprintf('Designing DECOUPLED airframe controllers with ROBUST tuning...\n');

for i = 1:num_points
    % Extract current flight parameters
    t = time_vector(i);
    m = mass_vector(i);
    V = velocity_vector(i);
    rho = density_vector(i);
    T = thrust_vector(i);
    I = inertia_vector(i);

    fprintf('--- Designing and Analyzing for t = %.1f s ---\n', t);
    
    % Recalculate derived constants
    d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);
    d2 = T * L_arm / I;
    d3 = (rho * V^2 * S * L)/(2 * I) * Cma;
    d4 = (rho * V * S * L^2)/(2 * I) * Cmq;
    d5 = T / (m * V);
    
    % Build the 3-state AIRFRAME-ONLY model
    A_airframe = [-d1 0 1; 0 0 1; d3 0 d4];
    B_airframe = [-d5; 0; d2];
    
    % Design the LQR controller for the airframe using the new robust R
    K_current = lqr(A_airframe, B_airframe, Q, R);
    K_gains(i, :) = K_current;

    % Stability Analysis on the Airframe Control Loop
    A_closed_loop = A_airframe - B_airframe * K_current;
    closed_loop_poles = eig(A_closed_loop);
    stability_results(i, 1) = max(real(closed_loop_poles));

    loop_transfer_sys = ss(A_airframe, B_airframe, K_current, 0);
    [GM, PM] = margin(loop_transfer_sys);
    stability_results(i, 2) = 20*log10(GM);
    stability_results(i, 3) = PM;

    fprintf('   -> Max Real(Eigenvalue): %.3f | Gain Margin: %.2f dB | Phase Margin: %.2f deg\n', ...
        stability_results(i,1), stability_results(i,2), stability_results(i,3));
end

%% 5. Display Final Results and Implementation Notes
fprintf('\n--- Airframe Controller Design Complete ---\n\n');
disp('Time Vector (s):');
disp(time_vector');

disp('Scheduled Airframe Gain Matrix K (3-state):');
disp(K_gains);

fprintf('\n--- Stability Analysis Summary ---\n\n');
results_table = table(time_vector', stability_results(:,1), stability_results(:,2), stability_results(:,3), ...
    'VariableNames', {'Time_s', 'Max_Real_Eigenvalue', 'GainMargin_dB', 'PhaseMargin_deg'});
disp(results_table);

fprintf('\n--- IMPLEMENTATION IN SIMULINK ---\n');
fprintf('1. Use these 3-state gains in your lookup table.\n');
fprintf('2. The input to the gain block is the airframe state vector: [alpha, theta, theta_dot].\n');
fprintf('3. The output of the gain block is the TVC command, delta_c.\n');
fprintf('4. This delta_c is now the input to your separate 2-state actuator model.\n');
