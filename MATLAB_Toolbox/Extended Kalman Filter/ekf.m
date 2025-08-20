clc; clear; close all
% EKF

% Load Sensor Data
load_data

a_B = [Sensors.ax, Sensors.ay, Sensors.az]';
x_NED_gps = [Sensors.N_m, Sensors.E_m, Sensors.D_m]';
v_NED_gps = [Sensors.V_N_mps, Sensors.V_E_mps, Sensors.V_D_mps]';

pqr_gyro = [Sensors.p, Sensors.q, Sensors.r]';

mag_B_meas = [Sensors.mX_uT, Sensors.mY_uT, Sensors.mZ_uT]';

g_I_ref = [0 0 9.81]';

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

% Gyroscope noise parameter (Angular Random Walk)
% Assuming units are in rad/s/sqrt(Hz)
ARW_P = 6.0e-3;
ARW_Q = ARW_P;
ARW_R = ARW_P;
% 1. Form the continuous-time process noise covariance matrix (Q_c).
% The diagonal elements are the Power Spectral Density (PSD) of the noise,
% which is the square of the ARW values.
Q = diag([ARW_P^2, ARW_Q^2, ARW_R^2]);

M = zeros(3, 3, N);
A_d = zeros(3, 3, N);
B_d = zeros(3, 3, N);

% H(k) is the output matrix. Since the output vector is the 'measured' roll
% pitch and yaw, the output matrix is is simply the Identity matrix
H = eye(3);

I = eye(3);

% R is the measurement noise covarience matrix
% R is the measurement noise covariance matrix
R_variances = [3.1874e-05, 3.1439e-05, 0.0027];
R = diag(R_variances);

% --- FIX: Add a small regularization term to prevent singularity ---
R = R + eye(3) * 1e-8; 

% L is the Kalman filter gain
L = zeros(3, 3, N);

% P matrix: corrected attitude covarience matrix
P = zeros(3, 3, N);
P(:, :, 1) = eye(3) * 0.1;

a_I_meas = zeros(3, N);
a_B_meas = zeros(3, N);
g_B_meas = zeros(3, N);


for k = 2:N


    % Extract Euler angles at k-1
    phi_prev   = euler321_hat(1, k-1);
    theta_prev = euler321_hat(2, k-1);
    psi_prev   = euler321_hat(3, k-1);

    if isnan(phi_prev)
        disp("NAN atk = :");
        disp(k);
    end
    
    [C_IB, C_BI] = euler321_to_CIB_CBI(phi_prev, theta_prev, psi_prev);
    
    % Specific force to NED: use body->inertial
    f_NED = C_BI * a_B(:, k);         % 3x1
    
    % Inertial acceleration (subtract gravity in NED)
    a_NED_bar(:, k) = f_NED - g_I_ref;      % 3x1
    
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

    P_gyro = pqr_gyro(1, k);
    Q_gyro = pqr_gyro(2, k);
    R_gyro = pqr_gyro(3, k);


    A_d(1, 1, k) = 1 + dt*cos(phi_hat)*tan(theta_hat)*Q_gyro - ... 
                    dt*sin(phi_hat)*tan(theta_hat)*R_gyro;
    A_d(1, 2, k) = (sin(phi_hat)/cos(theta_hat)^2)*Q_gyro - ...
                    dt*cos(phi_hat)*cos(theta_hat)^2*R_gyro;
    A_d(1, 3, k) = 0;
    A_d(2, 1, k) = -dt*sin(phi_hat)*Q_gyro - dt*cos(phi_hat)*R_gyro;
    A_d(2, 2, k) = 1;
    A_d(2, 3, k) = 0;
    A_d(3, 1, k) = dt*(cos(phi_hat)/cos(theta_hat))*Q_gyro - ...
                    dt*sin(phi_hat)*cos(theta_hat)*R_gyro;
    A_d(3, 2, k) = dt*(sin(phi_hat)*tan(theta_hat)/cos(theta_hat))*Q_gyro + ...
                    dt*(cos(phi_hat)*tan(theta_hat)/cos(theta_hat))*R_gyro;
    A_d(3, 3, k) = 1;

    B_d(:, :, k) = [...
            1 sin(phi_hat)*tan(theta_hat) cos(phi_hat)*tan(theta_hat);
            0 cos(phi_hat) -sin(phi_hat);
            0 sin(phi_hat)*sec(theta_hat) cos(phi_hat)*sec(theta_hat) ...
            ] * dt;
    
    % M(k) is the attitude estimation error covariance matrix, propagated
    % to this sample instant
    M(:, :, k) = A_d(:, :, k)*P(:, :, k-1)*A_d(:, :, k)' + B_d(:, :, k)*Q*B_d(:, :, k)';

    % Step 6: Calculate the Kalman filter gain
    S = H * M(:, :, k) * H' + R;
    L(:, :, k) = M(:, :, k) * H' / S;



    % Step 7: Calc Measured Gravity and Magnetic Field Vectors in Body Axis
    a_I_meas(:, k) = (v_NED_hat(:, k) - v_NED_hat(:, k-1))/dt;
    a_B_meas(:, k) = C_IB * a_I_meas(:, k);

    g_B_meas(:, k) = a_B_meas(:, k) + a_B(:, k);
    
    % m_B_meas = [m_Bx m_By m_Bz]' (already have)

    % Step 8: Obtain Reference Gravity and Mag field Vectors
    
    % Will change with coords and alt later
    g_I_ref = g_I_ref; 
    mag_I_ref = [ 27.5550, -2.4169, -16.0849 ]';
    
    % Step 9: Calculate the "Measured Altitude from the Measured and
    % Reference Vectors

    [phi_meas, theta_meas, psi_meas] = ...
        triad(g_I_ref, mag_I_ref, g_B_meas(:, k), mag_B_meas(:, k));
    euler_meas = [phi_meas theta_meas psi_meas]';
    
    % Step 10: Correct Attitude Estimate based on Sensor Measurements
    euler321_hat(:, k) = euler321_bar(:, k) + ... 
                        L(:, :, k) * (euler_meas - euler321_bar(:, k));
    
    % Step 11: Correct Attitude Estimation Error Covarience Matrix
    P(:, :, k) = (I - L(:, :, k)*H)*M(:, :, k) * (I - L(:, :, k)*H)' + ...
                                    L(:, :, k)*R*L(:, :, k)';

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

% --- Plotting Attitude Estimates vs. Reference ---
figure;
hold on;

% Convert the reference time to a numeric vector of seconds
t0 = TT_ref_att_rs.Time(1); 
t_ref_att_s = seconds(TT_ref_att_rs.Time - t0);

% Plot Estimates (solid lines)
plot(t_s, euler321_hat(1, :)*180/pi, 'r-', 'LineWidth', 1.5); % Roll
plot(t_s, euler321_hat(2, :)*180/pi, 'g-', 'LineWidth', 1.5); % Pitch
plot(t_s, euler321_hat(3, :)*180/pi, 'b-', 'LineWidth', 1.5); % Yaw

% Plot References (dashed lines)
plot(t_ref_att_s, TT_ref_att_rs.phi_ref*180/pi, 'r--', 'LineWidth', 1);
plot(t_ref_att_s, TT_ref_att_rs.theta_ref*180/pi, 'g--', 'LineWidth', 1);
plot(t_ref_att_s, TT_ref_att_rs.psi_ref*180/pi, 'b--', 'LineWidth', 1);

% Formatting
grid on;
title('EKF Attitude Estimates vs. Reference');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll_{est}', 'Pitch_{est}', 'Yaw_{est}', 'Roll_{ref}', 'Pitch_{ref}', 'Yaw_{ref}');
hold off;


% --- Plotting Attitude Estimation Errors ---
figure;
hold on;

% Resample reference data onto the EKF time grid to calculate error
phi_ref_interp = interp1(t_ref_att_s, TT_ref_att_rs.phi_ref, t_s);
theta_ref_interp = interp1(t_ref_att_s, TT_ref_att_rs.theta_ref, t_s);
psi_ref_interp = interp1(t_ref_att_s, TT_ref_att_rs.psi_ref, t_s);

% Calculate errors
error_phi = (euler321_hat(1, :)' - phi_ref_interp) * 180/pi;
error_theta = (euler321_hat(2, :)' - theta_ref_interp) * 180/pi;
error_psi = (euler321_hat(3, :)' - psi_ref_interp) * 180/pi;

% Plot Errors
plot(t_s, error_phi, 'r-');
plot(t_s, error_theta, 'g-');
plot(t_s, error_psi, 'b-');

% Formatting
grid on;
title('EKF Attitude Estimation Error');
xlabel('Time (s)');
ylabel('Error (degrees)');
legend('Roll Error', 'Pitch Error', 'Yaw Error');
hold off;
