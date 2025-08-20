clc; clear; close all
%% EKF

% Load Sensor Data
load_data

a_B = [Sensors.ax, Sensors.ay, Sensors.az]';
x_NED_gps = [Sensors.N_m, Sensors.E_m, Sensors.D_m]';
v_NED_gps = [Sensors.V_N_mps, Sensors.V_E_mps, Sensors.V_D_mps]';

pqr_gyro = [Sensors.p, Sensors.q, Sensors.r]';

g_ned = [0; 0; 9.81];

N = length(Sensors.t_us);
dt = double(Sensors.t_us(2)) * 10.0^-6;

t_s = double(Sensors.t_us) * 10.0^-6;

a_NED_bar = zeros(3, N);
v_NED_bar = zeros(3, N);
x_NED_bar = zeros(3, N);

a_NED_hat = zeros(3, N);
v_NED_hat = zeros(3, N);
x_NED_hat = zeros(3, N);

euler321_bar = zeros(3, N);
euler321_hat = zeros(3, N);

euler_rates_measured = zeros(3, N);

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
% Propagate Attitude Estimation Error Covariance Matrix Setup
%     M(k) = A_d*P(k-1)*A_d' + B_d*Q*B_d';

M = zeros(3, 3, N);
Q = zeros(3, 3, N);
A_d = zeros(3, 3, N);
B_d = zeros(3, 3, N);


for k = 2:N


    % Extract Euler angles at k-1
    phi_prev   = euler321_hat(1, k-1);
    theta_prev = euler321_hat(2, k-1);
    psi_prev   = euler321_hat(3, k-1);
    
    [C_IB, C_BI] = euler321_to_CIB_CBI(phi_prev, theta_prev, psi_prev);
    
    % Specific force to NED: use body->inertial
    f_NED = C_BI * a_B(:, k);         % 3x1
    
    % Inertial acceleration (subtract gravity in NED)
    a_NED_bar(:, k) = f_NED - g_ned;      % 3x1
    
    % Integrate acceleration to velocity and position
    v_NED_bar(:, k) = v_NED_bar(:, k-1) + a_NED_bar(:, k) * dt;
    x_NED_bar(:, k) = x_NED_bar(:, k-1) + v_NED_bar(:, k) * dt;
    
    
    % GPS Correction Step
    if ~isnan(x_NED_gps(1, k))
        % Position
        x_NED_hat(1, k) = x_NED_gps(1, k) + L_N(1, 1)*(x_NED_bar(1, k) - x_NED_gps(1, k))...
            + L_N(1, 2)*(v_NED_bar(1, k) - v_NED_gps(1, k));

        x_NED_hat(2, k) = x_NED_gps(2, k) + L_E(1, 1)*(x_NED_bar(2, k) - x_NED_gps(2, k))...
            + L_E(1, 2)*(v_NED_bar(2, k) - v_NED_gps(2, k));
        
        x_NED_hat(3, k) = x_NED_gps(3, k) + L_D(1, 1)*(x_NED_bar(3, k) - x_NED_gps(3, k))...
            + L_D(1, 2)*(v_NED_bar(3, k) - v_NED_gps(3, k));

        % Velocity
        v_NED_hat(1, k) = v_NED_gps(1, k) + L_N(2, 1)*(x_NED_bar(1, k) - x_NED_gps(1, k))...
            + L_N(2, 2)*(v_NED_bar(1, k) - v_NED_gps(1, k));

        v_NED_hat(2, k) = v_NED_gps(2, k) + L_E(2, 1)*(x_NED_bar(2, k) - x_NED_gps(2, k))...
            + L_E(2, 2)*(v_NED_bar(2, k) - v_NED_gps(2, k));
        
        v_NED_hat(3, k) = v_NED_gps(3, k) + L_D(2, 1)*(x_NED_bar(3, k) - x_NED_gps(3, k))...
            + L_D(2, 2)*(v_NED_bar(3, k) - v_NED_gps(3, k));
    
    else
        x_NED_hat(:, k) = x_NED_bar(:, k);
        v_NED_hat(:, k) = v_NED_bar(:, k);
    end

    x_NED_bar(:, k) = x_NED_hat(:, k);
    v_NED_bar(:, k) = v_NED_hat(:, k);

    % Attitude Step
    C_pqr = pqr_to_euler_rate(phi_prev, theta_prev);

    euler_rates_measured(:, k) = C_pqr * pqr_gyro(:, k);
    euler321_bar(:, k) = euler321_bar(:, k-1) + euler_rates_measured(:, k) * dt;

    % Propagate Attitude Estimation Error Covariance Matrix
    phi_hat = euler321_hat(1, k - 1);
    theta_hat = euler321_hat(2, k - 1);
    psi_hat = euler321_hat(3, k - 1);

    A_d(1, 1, k) = 1 + dt*cos(phi_hat)*tan(theta_hat)*Q - ... 
                    dt*sin(phi_hat)*tan(theta_hat)*R;
    A_d(1, 2, k) = (sin(phi_hat)/cos(theta_hat)^2)*Q - ...
                    dt*cos(phi_hat)*cos(theta_hat)^2*R;
    A_d(1, 3, k) = 0;
    A_d(2, 1, k) = -dt*sin(phi_hat)*Q - dt*cos(phi_hat)*R;
    A_d(2, 2, k) = 1;
    A_d(2, 3, k) = 0;
    A_d(3, 1, k) = dt*(cos(phi_hat)/cos(theta_hat))*Q - ...
                    dt*sin(phi_hat)*cos(theta_hat)*R;
    A_d(3, 2, k) = dt*(sin(phi_hat)*tan(theta_hat)/cos(theta_hat))*Q + ...
                    dt*(cos(phi_hat)*tan(theta_hat)/cos(theta_hat))*R;
    A_d(3, 3, k) = 1;

    B_d(:, k) = [...
            1 sin(phi_hat)*tan(theta_hat) cos(phi_hat)*tan(theta_hat);
            0 cos(phi_hat) -sin(phi_hat);
            0 sin(phi_hat)*sec(theta_hat) cos(phi_hat)*sec(theta_hat) ...
            ] * dt;

    M(k) = A_d*P(k-1)*A_d' + B_d*Q*B_d';


end


% --- Ensure vectors are columns and same x type ---
t_s_col = t_s(:);                                % 8001x1 seconds (numeric)
N_hat    = x_NED_hat(1,:).';                     % 8001x1 estimated N

% Convert timetable time (datetime) to seconds since same t0 used for t_s
% If t_s is already seconds since TT_accel.Time(1) (t0), then:
t0 = TT_accel.Time(1);
t_ref_s = seconds(TT_ref_Xe_rs.Time - t0);       % numeric seconds
N_ref    = TT_ref_Xe_rs.N_ref;                   % N x 1

% If lengths differ, resample reference onto t_s grid
N_ref_on_ts = interp1(t_ref_s, N_ref, t_s_col, 'linear', 'extrap');

% --- Plot together using same x-axis and lengths ---
figure; 
plot(t_s_col, N_hat, 'b-', 'LineWidth', 1.5); hold on;
plot(t_s_col, N_ref_on_ts, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('North position N (m)');
legend('N_{hat}', 'N_{ref} @ IMU rate', 'Location', 'best');
title('Estimated vs Reference N');
hold off;


figure; 
plot(t_s_col, N_hat - N_ref_on_ts, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('North position Error N (m)');
legend('N_{hat} error', 'Location', 'best');
title('Estimated vs Reference N Error');

figure;
plot(t_s, euler321_bar(:, :)*180/pi)