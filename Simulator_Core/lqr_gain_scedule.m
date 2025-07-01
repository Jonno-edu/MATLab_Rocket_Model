

%% 1. Define Fixed System Parameters
% These parameters do not change significantly with time.
S = 0.200296;           % Reference area (m²)
L = 9.542;              % Reference length (m)
L_arm = 3.9;            % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients (assuming constant for this design)
CLa = 2.0;
Cma = 0.885; % Unstable
Cmq = -1.05;

% Actuator model
natural_frequency = 62; % rad/s
damping_ratio = 0.5858;
wn = natural_frequency;
zeta = damping_ratio;

% Manually define actuator state-space to ensure states are [pos; rate]
A_act = [0 1; -wn^2 -2*zeta*wn];
B_act = [0; wn^2];
C_act = [1 0]; % Output is position

%% 2. Define Time-Varying Flight Data
% Data extracted from your Simulink plots at 6 key time points.
time_vector =      [0,     35,     65,     90,     125,    145];    % Scheduling variable (s)
velocity_vector =  [1.0,   150.6,  387.6,  728.5,  1604.7, 2523.6]; % m/s (using 1.0 at t=0 to avoid division by zero)
density_vector =   [1.225,   0.975,    0.4,    0.05,      0.0135,      0];      % kg/m^3
mass_vector =      [1873,  1528.1, 1228.4, 977.8,  628.3,  415.1];  % kg
thrust_vector =    [2.384e4,2.48e4, 2.67e4, 2.75e4, 2.76e4, 2.76e4];      % N
inertia_vector =   [3.31e4,2.77e4, 2.31e4, 1.93e4, 1.36e4, 1.05e4]; % kg*m^2

%% 3. Define Scheduled LQR Tuning Weights
% --- NEW SECTION: DEFINE TUNING FOR EACH REGIME ---

% Q matrix penalties are kept constant for consistency
max_alpha     = 3*pi/180; 
max_theta     = 4*pi/180;  
max_theta_dot = 0.1*pi/180;  
max_act_pos   = 1*pi/180;
max_act_rate  = 0.1*pi/180;  

Q_diag = [1/max_alpha^2, 1/max_theta^2, 1/max_theta_dot^2, 1/max_act_pos^2, 1/max_act_rate^2];
Q = diag(Q_diag);

% R matrix penalties will be scheduled based on flight regime
% A smaller max_TVC_command means a larger R penalty (gentler controller)
max_TVC_liftoff = 0.5*pi/180;   % Gentle for t < 35s
max_TVC_maxQ    = 1*pi/180;   % Aggressive for 35s <= t < 90s
max_TVC_highAlt = 0.1*pi/180;  % Very gentle for t >= 90s (low damping)


%% 4. Main Loop: Design Controller for Each Time Point
num_points = length(time_vector);
K_gains = zeros(num_points, 5); % Pre-allocate matrix to store all K vectors

fprintf('Designing gain-scheduled LQR controllers...\n');

for i = 1:num_points
    % Extract current flight parameters
    t = time_vector(i);
    m = mass_vector(i);
    V = velocity_vector(i);
    rho = density_vector(i);
    T = thrust_vector(i);
    I = inertia_vector(i);
    
    fprintf('Designing controller for t = %.1f s\n', t);

    % --- NEW SECTION: SELECT R PENALTY BASED ON TIME ---
    if t < 35
        fprintf('   -> Using Liftoff/Low Speed tuning (Gentle)\n');
        R = 1/max_TVC_liftoff^2;
    elseif t >= 35 && t < 80
        fprintf('   -> Using Max Q Region tuning (Aggressive)\n');
        R = 1/max_TVC_maxQ^2;
    else % t >= 90
        fprintf('   -> Using High Altitude tuning (Very Gentle)\n');
        R = 1/max_TVC_highAlt^2;
    end
    
    % Recalculate derived constants for the current flight condition
    q = 0.5 * rho * V^2;
    d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);
    d2 = T * L_arm / I;
    d3 = (rho * V^2 * S * L)/(2 * I) * Cma;
    d4 = (rho * V * S * L^2)/(2 * I) * Cmq;
    d5 = T / (m * V);
    
    % Rebuild the plant model for the current flight condition
    A_plant = [-d1 0 1; 0 0 1; d3 0 d4];
    B_plant = [-d5; 0; d2];
    
    % Rebuild the combined open-loop system model
    A_current = [[A_plant, B_plant*C_act]; [zeros(2,3), A_act]];
    B_current = [B_plant*0; B_act]; 
    
    % Design the LQR controller for this specific point using the selected R
    K_current = lqr(A_current, B_current, Q, R);
    
    % Store the resulting gain vector
    K_gains(i, :) = K_current;
end

%% 5. Display Final Results
fprintf('\n--- Gain Scheduling Design Complete ---\n\n');
disp('Time Vector (s) for Lookup Table Breakpoints:');
disp(time_vector');
disp('Scheduled Gain Matrix K (each row is a K vector for a time point):');
disp(K_gains);
fprintf('\nImplementation instructions remain the same.\n');


