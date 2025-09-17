clc; clear; close all
% EKF

% Load Sensor Data
load_data

%% Run EKF

a_B_raw = [Sensors.ax, Sensors.ay, Sensors.az]';
a_B = - a_B_raw;

mag_B_meas = [Sensors.mX_uT, Sensors.mY_uT, Sensors.mZ_uT]';

% GPS Variables
x_NED_gps = [Sensors.N_m, Sensors.E_m, Sensors.D_m]';
v_NED_gps = [Sensors.V_N_mps, Sensors.V_E_mps, Sensors.V_D_mps]';

% LPF GPS + Accel
dt_gps = 1/20;
fc_gps = 0.5; % Hz
rc_gps = 1/(2*pi*fc_gps);
lp_a_gps = dt_gps/(rc_gps + dt_gps);

dt_accel = 1/2000;
fc_accel = 0.5; % Hz
rc_accel = 1/(2*pi*fc_accel);
lp_a_accel = dt_accel/(rc_accel + dt_accel);

mag_B_lpf = zeros(3, N);
mag_B_lpf(:, 1) = [ 27.5550, -2.4169, -16.0849 ];
a_B_lpf = zeros(3, N);
a_B_lpf(:, 1) = [0, 0, 9.81];

v_NED_lpf = zeros(3, N);
a_NED_lpf = zeros(3, N);


g_I_ref = [0 0 9.81]';


pqr_gyro = [Sensors.p, Sensors.q, Sensors.r]';

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

euler321_GPS_estimate = zeros(3, N);

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

% L is the Kalman filter gain
L = zeros(3, 3, N);

% P matrix: corrected attitude covarience matrix
P = zeros(3, 3, N);
P(:, :, 1) = eye(3) * 0.1;

a_I_meas = zeros(3, N);
a_B_meas = zeros(3, N);
g_B_meas = zeros(3, N);

last_gps_update_k = 1;


for k = 2:N

    % Accelerometer LPF

    a_B_lpf(:, k) = lp_a_accel*a_B_raw(:, k) + (1-lp_a_accel)*a_B_lpf(:, k - 1);
    mag_B_lpf(:, k) = lp_a_accel*mag_B_meas(:, k) + (1-lp_a_accel)*mag_B_lpf(:, k - 1);


    % Extract Euler angles at k-1
    phi_prev   = euler321_hat(1, k-1);
    theta_prev = euler321_hat(2, k-1);
    psi_prev   = euler321_hat(3, k-1);
    
    [C_IB, C_BI] = euler321_to_CIB_CBI(phi_prev, theta_prev, psi_prev);
    
    % Specific force to NED: use body->inertial
    a_NED_gyro = C_BI * a_B(:, k);         % 3x1
    
    % Inertial acceleration (subtract gravity in NED)
    a_NED_bar(:, k) = a_NED_gyro + g_I_ref;      % 3x1

    
    % Integrate acceleration to velocity and position
    v_NED_bar(:, k) = v_NED_bar(:, k-1) + a_NED_bar(:, k) * dt;
    x_NED_bar(:, k) = x_NED_bar(:, k-1) + v_NED_bar(:, k) * dt;
    
    
    % GPS Correction Step
    if ~isnan(x_NED_gps(1, k))
        % Position Innovation
        pos_innovation_N = x_NED_gps(1, k) - x_NED_bar(1, k);
        pos_innovation_E = x_NED_gps(2, k) - x_NED_bar(2, k);
        pos_innovation_D = x_NED_gps(3, k) - x_NED_bar(3, k);

        % Velocity Innovation
        vel_innovation_N = v_NED_gps(1, k) - v_NED_bar(1, k);
        vel_innovation_E = v_NED_gps(2, k) - v_NED_bar(2, k);
        vel_innovation_D = v_NED_gps(3, k) - v_NED_bar(3, k);

        % Corrected Position (hat)
        x_NED_hat(1, k) = x_NED_bar(1, k) + L_N(1, 1) * ...
                    pos_innovation_N + L_N(1, 2) * vel_innovation_N;

        x_NED_hat(2, k) = x_NED_bar(2, k) + L_E(1, 1) * ...
                    pos_innovation_E + L_E(1, 2) * vel_innovation_E;
        
        x_NED_hat(3, k) = x_NED_bar(3, k) + L_D(1, 1) * ...
                    pos_innovation_D + L_D(1, 2) * vel_innovation_D;

        % Corrected Velocity (Hat)
        v_NED_hat(1, k) = v_NED_bar(1, k) + L_N(2, 1) * ...
                    pos_innovation_N + L_N(2, 2) * vel_innovation_N;

        v_NED_hat(2, k) = v_NED_bar(2, k) + L_E(2, 1) * ...
                    pos_innovation_E + L_E(2, 2) * vel_innovation_E;

        v_NED_hat(3, k) = v_NED_bar(3, k) + L_D(2, 1) * ...
                    pos_innovation_D + L_D(2, 2) * vel_innovation_D;
        

    
    else
        x_NED_hat(:, k) = x_NED_bar(:, k);
        v_NED_hat(:, k) = v_NED_bar(:, k);
        

    end

    x_NED_bar(:, k) = x_NED_hat(:, k);
    v_NED_bar(:, k) = v_NED_hat(:, k);

    % Attitude Step
    C_pqr = pqr_to_euler_rate(phi_prev, theta_prev);

    euler_rates_measured(:, k) = C_pqr * pqr_gyro(:, k);
    euler321_bar(:, k) = euler321_hat(:, k-1) + euler_rates_measured(:, k) * dt;

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

    % --- Corrected Code Block ---
    
    % Check if a new GPS velocity measurement is available for the current step k
    if ~isnan(v_NED_gps(1, k))
        % Calculate the number of time steps since the last GPS update
        gps_ticks_diff = k - last_gps_update_k;
        
        % Only calculate new acceleration if time has passed
        if gps_ticks_diff > 0
            dt_gps = dt * gps_ticks_diff;
            % Calculate inertial acceleration by differentiating GPS velocity
            v_NED_lpf(:, k) = lp_a_gps * v_NED_gps(:, k) + (1-lp_a_gps)*v_NED_lpf(:, k - 1);
            a_NED_lpf(:, k) = (mag_B_meas(:, k) - v_NED_lpf(:, k - 1))/dt_gps;
    
            % Transform the total inertial acceleration into the body frame
            a_B_meas(:, k) = C_IB * a_NED_lpf(:, k);
            
            % Subtract the raw (specific force) accelerometer reading to isolate the gravity vector
            g_B_meas(:, k) = a_B_meas(:, k) - a_B_lpf(:, k);
            
            % m_B_meas = [m_Bx m_By m_Bz]' (already have)
        
            % Step 8: Obtain Reference Gravity and Mag field Vectors
            
            % Will change with coords and alt later
            % g_I_ref = g_I_ref; 
            mag_I_ref = [ 27.5550, -2.4169, -16.0849 ]';
            
            % Step 9: Calculate the "Measured Altitude from the Measured and
            % Reference Vectors
        
            [phi_meas, theta_meas, psi_meas] = ...
                triad(g_I_ref, mag_I_ref, g_B_meas(:, k), mag_B_lpf(:, k));
        
            euler_meas = [phi_meas theta_meas psi_meas]';
            euler321_GPS_estimate(:, k) = euler_meas;



            % Update the timestamp of the last successful GPS update
            last_gps_update_k = k;
        else
            % If it's the same tick, no new info; hold the previous acceleration value
            a_I_meas(:, k) = a_I_meas(:, k-1);
            euler321_GPS_estimate(:, k) = euler321_GPS_estimate(:, k - 1);
        end
    else
        % If no new GPS data is available at this step, hold the last known acceleration
        if k > 1
            v_NED_lpf(:, k) = v_NED_lpf(:, k-1);
            a_I_meas(:, k) = a_I_meas(:, k-1);
        else
            % Handle the initial case where no prior data exists
            a_I_meas(:, k) = [0; 0; 0];
        end
    end

    

    % Step 10: Correct Attitude Estimate based on Sensor Measurements
    euler321_hat(:, k) = euler321_bar(:, k) + ... 
                        L(:, :, k) * (euler_meas - euler321_bar(:, k));
    
    % Step 11: Correct Attitude Estimation Error Covarience Matrix
    P(:, :, k) = (I - L(:, :, k)*H)*M(:, :, k) * (I - L(:, :, k)*H)' + ...
                                    L(:, :, k)*R*L(:, :, k)';
    

end



% --- Plotting Velocity and Velocity Error ---

% Plot Estimated Velocity Components
figure;
subplot(2,1,1); % Create subplot for velocity
hold on;
plot(t_s, v_NED_hat(3, :), 'r-');
plot(t_s, TT_ref_Ve_rs.V_D_ref,'b-');



error_Vn = v_NED_hat(1, :)' - TT_ref_Ve_rs.V_N_ref;
error_Ve = v_NED_hat(2, :)' - TT_ref_Ve_rs.V_E_ref;
error_Vd = v_NED_hat(3, :)' - TT_ref_Ve_rs.V_D_ref;

% Create subplot for velocity error
subplot(2,1,2);
hold on;
plot(t_s, error_Vd, 'r-');
% plot(t_s, error_Ve, 'g-');
% plot(t_s, error_Vd, 'b-');
grid on;
title('EKF Velocity Estimation Error');
xlabel('Time (s)');
ylabel('Error (m/s)');
legend('North Error');
hold off;

figure;
plot(t_s, euler321_bar(2, :)*180/pi, 'r-'); hold on;
plot(t_s, euler321_hat(2, :)*180/pi, 'r-'); hold on;
plot(t_s, euler321_GPS_estimate(2, :)*180/pi, 'r-'); hold on;
plot(t_s, TT_ref_att_rs.theta_ref*180/pi,'b-');hold off;
title('Attitude');


figure;
plot(t_s, a_I_meas(3, :), 'r-'); hold on;
plot(t_s, TT_ref_Ae_rs.aD_e_ref,'b-'); hold off;
title('Accel D');

figure;
plot(t_s, g_B_meas(3, :), 'r-'); hold on;
plot(t_s, g_B_perfect_resampled(3, :),'b-'); hold off;
title('Accel D');


error_Vn_GPS = v_NED_lpf(1, :)' - TT_ref_Ve_rs.V_N_ref;
figure;
plot(t_s, error_Vn_GPS);


figure;
plot(t_s, v_NED_lpf(1, :)); hold on;
plot(t_s, TT_ref_Ve_rs.V_N_ref); hold off;

figure;
plot(t_s, mag_B_meas(:, :)); hold on;
plot(t_s, mag_B_lpf(:, :)); hold off;

figure;
plot(t_s, a_B_raw(:, :)); hold on;
plot(t_s, a_B_lpf(:, :)); hold off;