% EKF (specific force input) with yaw locked to 0; roll and pitch free + NIS/NEES
clc; clear; close all;

%% Run Simulink if needed
if ~exist('Sim', 'var') || isempty(Sim)
    dt_gps = 1/1;
    dt_ekf = 1/1000;
    Sim = sim("simulate_sensors.slx");
end

%% Timebase
t  = Sim.accel.Time;
dt = t(2) - t(1);
N  = numel(t);
g  = 9.81;

%% References (N, E, D)
ref_pos_data    = squeeze(Sim.ref_Xe.Data);
ref_pos_n_data  = ref_pos_data(:, 1);
ref_pos_e_data  = ref_pos_data(:, 2);
ref_pos_d_data  = ref_pos_data(:, 3);
ref_vel_data    = squeeze(Sim.ref_Ve.Data);
ref_vel_n_data  = ref_vel_data(:, 1);
ref_vel_e_data  = ref_vel_data(:, 2);
ref_vel_d_data  = ref_vel_data(:, 3);
ref_euler_data  = squeeze(Sim.ref_euler.Data);
ref_phi         = ref_euler_data(:, 1);
ref_theta       = ref_euler_data(:, 2);
ref_psi         = ref_euler_data(:, 3);

%% Sensors
accel_data = squeeze(Sim.accel.Data)';
fx_meas = accel_data(:,1);
fy_meas = accel_data(:,2);
fz_meas = accel_data(:,3);
gyro_data  = squeeze(Sim.gyro.Data)';
pw_meas = gyro_data(:,1);
qw_meas = gyro_data(:,2);
rw_meas = gyro_data(:,3);

mag_data = squeeze(Sim.mag.Data)';
mag_x_meas = mag_data(:, 1);
mag_y_meas = mag_data(:, 2);
mag_z_meas = mag_data(:, 3);

gps_pos_data    = squeeze(Sim.gps_Xe.Data)';
gps_pos_n_data  = gps_pos_data(:, 1);
gps_pos_e_data  = gps_pos_data(:, 2);
gps_pos_d_data  = gps_pos_data(:, 3);
gps_vel_data    = squeeze(Sim.gps_Ve.Data)';
gps_vel_n_data  = gps_vel_data(:, 1);
gps_vel_e_data  = gps_vel_data(:, 2);
gps_vel_d_data  = gps_vel_data(:, 3);

%% Noise settings
gps_pos_noise_std = 2.5*1;
gps_alt_noise_std = 3.0*1;
gps_vel_noise_std = 0.1;
sigma_a_proc  = 0.2;
sigma_w_proc  = deg2rad(0.1);  % rad/s/sqrt(Hz)

%% EKF State: x = [p_N, p_E, p_D, v_N, v_E, v_D, q0, q1, q2, q3]'
x_hat = zeros(10, 1); x_hat(7) = 1;
P = eye(10) * 1.0;

% Q for p,v driven by white accel
var_v  = sigma_a_proc^2 * dt^2;
var_p  = 0.25 * sigma_a_proc^2 * dt^4;
cov_pv = 0.5 * sigma_a_proc^2 * dt^3;
Q_pv_1D = [var_p,  cov_pv; cov_pv, var_v];
Q_pv_3D = blkdiag(Q_pv_1D, Q_pv_1D, Q_pv_1D);

% R and H
R = diag([gps_pos_noise_std^2, gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2, gps_vel_noise_std^2]);
H = [eye(6), zeros(6, 4)];

% Logs
x_hist = zeros(N, 10);
P_hist = zeros(N, 10);

% Consistency logs
nis_list  = nan(N,1);
nees_list = nan(N,1);

I10 = eye(10);

% Helpers: Euler from quaternion, quaternion from roll-pitch-yaw (psi=0)
quat_to_euler = @(q) deal( ...
    atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2)), ... % roll
    asin( max(-1,min(1, 2*(q(1)*q(3) - q(4)*q(2)))) ), ...          % pitch
    atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2)) ...   % yaw
);
rpy_to_quat = @(phi,theta,psi) [ ...
    cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2); ...
    sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2); ...
    cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2); ...
    cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2) ...
]; % scalar-first

%% EKF Loop
for k = 1:N
    % --- Unpack ---
    p_current = x_hat(1:3);
    v_current = x_hat(4:6);
    q_current = x_hat(7:10);
    q0=q_current(1); q1=q_current(2); q2=q_current(3); q3=q_current(4);

    % --- Sensors ---
    f_b = [fx_meas(k); fy_meas(k); fz_meas(k)];
    omega = [pw_meas(k); qw_meas(k); rw_meas(k)];

    % --- Prediction ---
    % DCM (body -> nav)
    C_bn = [ q0^2+q1^2-q2^2-q3^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
             2*(q1*q2 + q0*q3),   q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1);
             2*(q1*q3 - q0*q2),   2*(q2*q3 + q0*q1),   q0^2 - q1^2 - q2^2 + q3^2 ];

    g_n = [0;0;g];
    a_n = g_n - C_bn * f_b;

    x_hat(1:3) = p_current + v_current*dt + 0.5*a_n*dt^2;
    x_hat(4:6) = v_current + a_n*dt;

    p=omega(1); q=omega(2); r=omega(3);
    ohm = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
    x_hat(7:10) = q_current + 0.5 * ohm * q_current * dt;
    x_hat(7:10) = x_hat(7:10) / norm(x_hat(7:10));

    % Enforce yaw=0 while keeping current roll/pitch
    [phi_k, theta_k, ~] = quat_to_euler(x_hat(7:10));
    x_hat(7:10) = rpy_to_quat(phi_k, theta_k, 0);
    x_hat(7:10) = x_hat(7:10) / norm(x_hat(7:10));

    % --- Covariance prediction ---
    F = eye(10);
    F(1:3, 4:6) = eye(3)*dt;

    fx = f_b(1); fy=f_b(2); fz=f_b(3);
    S_f = [   0  -fz   fy;
            fz    0  -fx;
           -fy   fx    0 ];
    % Sensitivity for a_n = g_n - C_bn f_b: +2*C_bn*[f]_x
    F(4:6, 8:10) = (+2 * C_bn * S_f) * dt;

    F(7:10, 7:10) = eye(4) + 0.5 * ohm * dt;

    % Process noise (gyro density -> discrete)
    qn = x_hat(7:10);
    Gq = [-qn(2) -qn(3) -qn(4);
           qn(1) -qn(4)  qn(3);
           qn(4)  qn(1) -qn(2);
          -qn(3)  qn(2)  qn(1)];
    Q_q = 0.25 * (Gq * ((sigma_w_proc^2 * dt) * eye(3)) * Gq');
    Q = blkdiag(Q_pv_3D, Q_q);

    P = F * P * F' + Q;

    % --- Update ---
    if ~isnan(gps_pos_n_data(k))
        z = [gps_pos_n_data(k); gps_pos_e_data(k); gps_pos_d_data(k);
             gps_vel_n_data(k); gps_vel_e_data(k); gps_vel_d_data(k)];
        y = z - H * x_hat;
        S = H * P * H' + R;
        K = P * H' / S;

        % Consistency: NIS for this update
        nis_list(k) = y' / S * y;

        x_hat = x_hat + K * y;

        % Normalize and re-enforce yaw=0 after update
        x_hat(7:10) = x_hat(7:10) / norm(x_hat(7:10));
        [phi_k, theta_k, ~] = quat_to_euler(x_hat(7:10));
        x_hat(7:10) = rpy_to_quat(phi_k, theta_k, 0);
        x_hat(7:10) = x_hat(7:10) / norm(x_hat(7:10));

        A = (I10 - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P');

        % Consistency: NEES for 6D pos+vel substate
        x_true_6 = [ref_pos_n_data(k); ref_pos_e_data(k); ref_pos_d_data(k); ...
                    ref_vel_n_data(k); ref_vel_e_data(k); ref_vel_d_data(k)];
        x_hat_6  = x_hat(1:6);
        P_6      = P(1:6,1:6);
        e6       = x_true_6 - x_hat_6;
        nees_list(k) = e6' / P_6 * e6;
    end

    % Log
    x_hist(k,:) = x_hat';
    P_hist(k,:) = diag(P)';
end

%% Extract and Convert States
est_p_n = x_hist(:,1); est_p_e = x_hist(:,2); est_p_d = x_hist(:,3);
est_v_n = x_hist(:,4); est_v_e = x_hist(:,5); est_v_d = x_hist(:,6);
est_q0  = x_hist(:,7); est_q1  = x_hist(:,8); est_q2  = x_hist(:,9); est_q3  = x_hist(:,10);

% Euler angles
N = numel(t);
est_phi = zeros(N, 1);
est_theta = zeros(N, 1);
est_psi = zeros(N, 1);
for k = 1:N
    q0 = est_q0(k); q1 = est_q1(k); q2 = est_q2(k); q3 = est_q3(k);
    est_phi(k) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sinp = 2*(q0*q2 - q3*q1);
    if abs(sinp) >= 1
        est_theta(k) = sign(sinp) * pi/2;
    else
        est_theta(k) = asin(sinp);
    end
    est_psi(k) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
end

%% Plots — All EKF States
sigma = sqrt(max(P_hist, 0));

figure('Name','Position (NED)'); tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on;
plot(t, ref_pos_n_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_p_n,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_pos_n_data,'r.','DisplayName','GPS');
ylabel('N (m)'); title('Position compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_p_n - ref_pos_n_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,1),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,1),'c--','DisplayName','-1\sigma');
ylabel('Error (m)'); title('N error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, ref_pos_e_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_p_e,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_pos_e_data,'r.','DisplayName','GPS');
ylabel('E (m)'); title('Position compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_p_e - ref_pos_e_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,2),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,2),'c--','DisplayName','-1\sigma');
ylabel('Error (m)'); title('E error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, ref_pos_d_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_p_d,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_pos_d_data,'r.','DisplayName','GPS');
xlabel('Time (s)'); ylabel('D (m)'); title('Position compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_p_d - ref_pos_d_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,3),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,3),'c--','DisplayName','-1\sigma');
xlabel('Time (s)'); ylabel('Error (m)'); title('D error'); legend('Location','best');

figure('Name','Velocity (NED)'); tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on;
plot(t, ref_vel_n_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_v_n,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_vel_n_data,'r.','DisplayName','GPS');
ylabel('v_N (m/s)'); title('Velocity compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_v_n - ref_vel_n_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,4),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,4),'c--','DisplayName','-1\sigma');
ylabel('Error (m/s)'); title('v_N error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, ref_vel_e_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_v_e,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_vel_e_data,'r.','DisplayName','GPS');
ylabel('v_E (m/s)'); title('Velocity compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_v_e - ref_vel_e_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,5),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,5),'c--','DisplayName','-1\sigma');
ylabel('Error (m/s)'); title('v_E error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, ref_vel_d_data,'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, est_v_d,       'b-','LineWidth',1.2,'DisplayName','EKF');
plot(t, gps_vel_d_data,'r.','DisplayName','GPS');
xlabel('Time (s)'); ylabel('v_D (m/s)'); title('Velocity compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, est_v_d - ref_vel_d_data,'m-','LineWidth',1.2,'DisplayName','Error');
plot(t, +sigma(:,6),'c--','DisplayName','+1\sigma');
plot(t, -sigma(:,6),'c--','DisplayName','-1\sigma');
xlabel('Time (s)'); ylabel('Error (m/s)'); title('v_D error'); legend('Location','best');

figure('Name','Euler Angles (deg)'); tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on;
plot(t, rad2deg(ref_phi),'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, rad2deg(est_phi),'b-','LineWidth',1.2,'DisplayName','EKF');
ylabel('Roll (deg)'); title('Euler compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, rad2deg(est_phi - ref_phi),'m-','LineWidth',1.2,'DisplayName','Error');
ylabel('Error (deg)'); title('Roll error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, rad2deg(ref_theta),'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, rad2deg(est_theta),'b-','LineWidth',1.2,'DisplayName','EKF');
ylabel('Pitch (deg)'); title('Euler compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, rad2deg(est_theta - ref_theta),'m-','LineWidth',1.2,'DisplayName','Error');
ylabel('Error (deg)'); title('Pitch error'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, rad2deg(ref_psi),'k-','LineWidth',1.2,'DisplayName','Ref');
plot(t, rad2deg(est_psi),'b-','LineWidth',1.2,'DisplayName','EKF');
xlabel('Time (s)'); ylabel('Yaw (deg)'); title('Euler compare'); legend('Location','best');
nexttile; hold on; grid on;
plot(t, rad2deg(est_psi - ref_psi),'m-','LineWidth',1.2,'DisplayName','Error');
xlabel('Time (s)'); ylabel('Error (deg)'); title('Yaw error'); legend('Location','best');

figure('Name','Quaternions'); tiledlayout(4,1,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on; plot(t, est_q0,'b-','LineWidth',1.2); ylabel('q0');
nexttile; hold on; grid on; plot(t, est_q1,'b-','LineWidth',1.2); ylabel('q1');
nexttile; hold on; grid on; plot(t, est_q2,'b-','LineWidth',1.2); ylabel('q2');
nexttile; hold on; grid on; plot(t, est_q3,'b-','LineWidth',1.2); ylabel('q3'); xlabel('Time (s)');

figure('Name','State Covariance (1\sigma)'); tiledlayout(5,2,'Padding','compact','TileSpacing','compact');
labels = {'p_N','p_E','p_D','v_N','v_E','v_D','q0','q1','q2','q3'};
for i = 1:10
    nexttile; hold on; grid on;
    plot(t, sigma(:,i),'c-','LineWidth',1.2);
    ylabel(['\sigma(',labels{i},')']);
    if i >= 9, xlabel('Time (s)'); end
end

%% Consistency (NIS/NEES) with 95% chi-square bounds
alpha = 0.95;
dof_m = 6;  % measurement dim
dof_x = 6;  % pos+vel substate
if exist('chi2inv','file')
    nis_lo = chi2inv((1-alpha)/2, dof_m);
    nis_hi = chi2inv(1-(1-alpha)/2, dof_m);
    nees_lo = chi2inv((1-alpha)/2, dof_x);
    nees_hi = chi2inv(1-(1-alpha)/2, dof_x);
else
    nis_lo = icdf('Chi-square',(1-alpha)/2, dof_m);
    nis_hi = icdf('Chi-square',1-(1-alpha)/2, dof_m);
    nees_lo = icdf('Chi-square',(1-alpha)/2, dof_x);
    nees_hi = icdf('Chi-square',1-(1-alpha)/2, dof_x);
end

valid = ~isnan(nis_list);
figure('Name','Consistency (NIS/NEES)'); tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; hold on; grid on;
plot(t(valid), nis_list(valid),'b-','LineWidth',1.2,'DisplayName','NIS');
yline(nis_lo,'r--','DisplayName','95% lo');
yline(nis_hi,'r--','DisplayName','95% hi');
title('NIS vs 95% bounds'); xlabel('Time (s)'); ylabel('NIS'); legend('Location','best');

valid2 = ~isnan(nees_list);
nexttile; hold on; grid on;
plot(t(valid2), nees_list(valid2),'m-','LineWidth',1.2,'DisplayName','NEES (pos+vel)');
yline(nees_lo,'r--','DisplayName','95% lo');
yline(nees_hi,'r--','DisplayName','95% hi');
title('NEES vs 95% bounds'); xlabel('Time (s)'); ylabel('NEES'); legend('Location','best');
