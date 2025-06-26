%% Plant model (TVC Rocket - with Actuator Model)
%clear; clc;

% Define Constants
% Physical parameters (SI units)
m_initial = 974;     % rocket initial mass (kg)
Isp = 250;           % Specific impulse (s)
g0 = 9.81;           % Sea-level gravity (m/s^2)
S = 0.200296;        % reference area (m²)
L = 9.542;           % reference length (m)
I = 19000;           % moment of inertia about pitch axis (kg⋅m²) - assumed constant
T = 23.6e3 * 0.5;    % thrust (N) - assumed constant
L_arm = 4;           % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients (per radian)
CLa = 2;             % lift curve slope (1/rad)
Cma = 2;             % moment curve slope (1/rad)
Cmq = -2.0;          % pitch damping coefficient (1/rad)

% Actuator Model Parameters
omega_n_act = 62;    % Natural frequency (rad/s)
zeta_act = 0.5858;   % Damping ratio 
actuator_gain = 1;   % Steady-state gain


%% Define Flight Operating Points
% Data extracted from flight simulation graphs
time_points     = [0, 6, 12, 18, 24, 30, 39, 48, 57, 63];
airspeed_points = [1, 150, 300, 450, 600, 750, 1000, 1250, 1550, 1580]; % V=0 causes singularity, so start with V=1
rho_points      = [1.22, 1.20, 1.10, 0.98, 0.82, 0.65, 0.38, 0.15, 0.05, 0.02];

num_points = length(time_points);
K_gains = zeros(num_points, 5); % Pre-allocate for storing gain matrices

% Calculate mass flow rate
mass_flow_rate = T / (Isp * g0);


%% LQR Design Loop for Each Operating Point
fprintf('--- Designing LQR controller for %d operating points ---\n\n', num_points);

for i = 1:num_points
    % Get current flight conditions
    t = time_points(i);
    V = airspeed_points(i);
    rho = rho_points(i);
    
    % Update mass based on time
    m = m_initial - mass_flow_rate * t;
    if m <= 0
        error('Mass calculation resulted in non-positive mass.');
    end

    % Recalculate d constants for the current operating point
    d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);
    d2 = T * L_arm / I;                               
    d3 = (rho * V^2 * S * L)/(2 * I) * Cma;         
    d4 = (rho * V * S * L^2)/(2 * I) * Cmq;        
    d5 = T / (m * V);                                 

    % Augmented State Space (Plant + Actuator)
    A_plant = [-d1 0 1;
               0   0 1;
               d3  0 d4];
    B_plant_to_actuator = [-d5; 0; d2];
    A_actuator = [0, 1;
                  -omega_n_act^2, -2*zeta_act*omega_n_act];
    B_actuator = [0; omega_n_act^2 * actuator_gain];
    
    A_augmented = [A_plant, B_plant_to_actuator, zeros(3,1);
                   zeros(2,3), A_actuator];
    B_augmented = [zeros(3,1); B_actuator];

    % Bryson's Rule for Q and R (remains constant for all points)
    Q_bryson_aug = diag([1/(5*pi/180)^2,    % Max 5° AoA acceptable
                         1/(5*pi/180)^2,    % Max 5° pitch acceptable  
                         1/(50*pi/180)^2,   % Max 50°/s pitch rate
                         1/(4*pi/180)^2,    % Max 4° actuator position
                         1/(4*pi/180)^2]);  % Max 4 rad/s actuator rate (updated from comment)

    R_bryson_aug = 1/(2*pi/180)^2;          % Max 2° nozzle command

    % Calculate LQR gain for the augmented system at this point
    K_lqr = lqr(A_augmented, B_augmented, Q_bryson_aug, R_bryson_aug);
    
    % Store the calculated gains
    K_gains(i, :) = K_lqr;
    
    fprintf('Point %d (t=%.1fs, V=%.0fm/s, rho=%.2fkg/m^3):\n', i, t, V, rho);
    fprintf('  K = [%.4f %.4f %.4f %.4f %.4f]\n\n', K_lqr);
end


%% Implementation in Simulink
% To implement this in Simulink, you would use a 1-D Lookup Table block.
% - The 'Table data' parameter would be the K_gains matrix.
% - The 'Breakpoints 1' parameter would be the scheduling variable, for example, 'time_points'.
% The block will then interpolate between the gain rows based on the input
% simulation time, providing a smooth transition of controller gains during flight.

disp('--- Gain Matrix for Simulink Lookup Table ---');
disp('This matrix contains the LQR gains. Each row corresponds to an operating point.');
disp(K_gains);

disp('--- Breakpoint Vector for Simulink Lookup Table ---');
disp('This vector contains the time points for scheduling.');
disp(time_points');
