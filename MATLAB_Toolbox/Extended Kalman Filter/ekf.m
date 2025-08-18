clc; clear; close all
%% EKF

% Load Sensor Data
load_data

a_B = [Sensors.ax, Sensors.ay, Sensors.az]';
x_NED_gps = [Sensors.N_m, Sensors.E_m, Sensors.D_m]';
v_NED_gps = [Sensors.V_N_mps, Sensors.V_E_mps, Sensors.V_D_mps]';

g_ned = [0; 0; 9.81];

N = length(Sensors.t_us);
dt = double(Sensors.t_us(2)) * 1.0 * 10.0^-6;

euler321 = zeros(3, N);
a_NED_prop = zeros(3, N);
v_NED_prop = zeros(3, N);
x_NED_prop = zeros(3, N);

N_hat = zeros(1, N);
E_hat = zeros(1, N);
D_hat = zeros(1, N);

vN_hat = zeros(1, N);
vE_hat = zeros(1, N);
vD_hat = zeros(1, N);

% Fixed L matrices
% L11: how hard to pull position estimate toward GPS position
% L21: how much to bleed position innovation into velocity (to arrest drift)
% L12: how much to let GPS velocity innovation correct position (usually 0 or very small)
% L22: how hard to pull velocity estimate toward GPS velocity

L_N = [ 0.2 0.0;
        0.05 0.1];
L_E = [ 0.2 0.0;
        0.05 0.1];
L_D = [ 0.1 0.0;
        0.02 0.05];


for k = 2:N


    % Extract Euler angles at k-1
    phi   = euler321(1, k-1);
    theta = euler321(2, k-1);
    psi   = euler321(3, k-1);
    
    [C_IB, C_BI] = euler321_to_CIB_CBI(phi, theta, psi);
    
    % Specific force to NED: use body->inertial
    f_NED = C_BI * a_B(:, k);         % 3x1
    
    % Inertial acceleration (subtract gravity in NED)
    a_NED_prop(:, k) = f_NED - g_ned;      % 3x1
    
    % Integrate acceleration to velocity and position
    v_NED_prop(:, k) = v_NED_prop(:, k-1) + a_NED_prop(:, k) * dt;
    x_NED_prop(:, k) = x_NED_prop(:, k-1) + v_NED_prop(:, k) * dt;
    
    
    % GPS Correction Step
    if ~isnan(x_NED_gps(1, k))
        % Position
        N_hat(k) = x_NED_gps(1, k) + L_N(1, 1)*(x_NED_prop(1, k) - x_NED_gps(1, k))...
            + L_N(1, 2)*(v_NED_prop(1, k) - v_NED_gps(1, k));

        E_hat(k) = x_NED_gps(2, k) + L_E(1, 1)*(x_NED_prop(2, k) - x_NED_gps(2, k))...
            + L_E(1, 2)*(v_NED_prop(2, k) - v_NED_gps(2, k));
        
        D_hat(k) = x_NED_gps(3, k) + L_D(1, 1)*(x_NED_prop(3, k) - x_NED_gps(3, k))...
            + L_D(1, 2)*(v_NED_prop(3, k) - v_NED_gps(3, k));

        % Velocity
        vN_hat(k) = v_NED_gps(1, k) + L_N(2, 1)*(x_NED_prop(1, k) - x_NED_gps(1, k))...
            + L_N(2, 2)*(v_NED_prop(1, k) - v_NED_gps(1, k));

        vE_hat(k) = v_NED_gps(2, k) + L_E(2, 1)*(x_NED_prop(2, k) - x_NED_gps(2, k))...
            + L_E(2, 2)*(v_NED_prop(2, k) - v_NED_gps(2, k));
        
        vD_hat(k) = v_NED_gps(3, k) + L_D(2, 1)*(x_NED_prop(3, k) - x_NED_gps(3, k))...
            + L_D(2, 2)*(v_NED_prop(3, k) - v_NED_gps(3, k));
    
    else
        N_hat(k) = x_NED_prop(1, k);
        E_hat(k) = x_NED_prop(2, k);
        D_hat(k) = x_NED_prop(3, k);
        vN_hat(k) = v_NED_prop(1, k);
        vE_hat(k) = v_NED_prop(2, k);
        vD_hat(k) = v_NED_prop(3, k);

    end

    x_NED_prop(:,k) = [N_hat(k); E_hat(k); D_hat(k)];
    v_NED_prop(:,k) = [vN_hat(k); vE_hat(k); vD_hat(k)];

end


