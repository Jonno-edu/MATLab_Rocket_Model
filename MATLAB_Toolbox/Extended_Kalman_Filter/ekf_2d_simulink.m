% EKF (specific force input) in North–Down; with accel/gyro bias states
clc; clear; close all;

%% Run Simulink if needed
if ~exist('Sim', 'var') || isempty(Sim)
    dt_gps = 1/5;
    dt_ekf = 1/1000;
    Sim = sim("simulate_sensors.slx");
end

%% Timebase
t  = Sim.accel.Time;
dt = t(2) - t(1);
N  = numel(t);
g  = 9.81;

%% References (N, E, D in Sim; use N and D here)
ref_pos_data    = squeeze(Sim.ref_Xe.Data);    % [N E D]
ref_pos_n_data  = ref_pos_data(:, 1);
ref_pos_d_data  = ref_pos_data(:, 3);
ref_vel_data    = squeeze(Sim.ref_Ve.Data);    % [N E D]
ref_vel_n_data  = ref_vel_data(:, 1);
ref_vel_d_data  = ref_vel_data(:, 3);
ref_euler_data  = squeeze(Sim.ref_euler.Data); % [roll pitch yaw]
ref_theta       = ref_euler_data(:, 2);        % pitch

%% Sensors
% Accelerometer outputs specific force in body: f_b = [fx fy fz]
accel_data = squeeze(Sim.accel.Data)';   % [fx fy fz] specific force (body)
gyro_data  = squeeze(Sim.gyro.Data)';    % [p q r] (body)
fx_meas = accel_data(:,1);               % body X specific force
fz_meas = accel_data(:,3);               % body Z specific force
qw_meas = gyro_data(:,2);                % body pitch rate

% GPS aligned to IMU rate with NaNs when absent
gps_pos_data    = squeeze(Sim.gps_Xe.Data)';    % [N E D]
gps_pos_n_data  = gps_pos_data(:, 1);
gps_pos_d_data  = gps_pos_data(:, 3);
gps_vel_data    = squeeze(Sim.gps_Ve.Data)';    % [N E D]
gps_vel_n_data  = gps_vel_data(:, 1);
gps_vel_d_data  = gps_vel_data(:, 3);

%% Noise settings
gps_noise_mult = 1;
gps_pos_noise_std = 2.5 * gps_noise_mult; % m (North)
gps_alt_noise_std = 3.0 * gps_noise_mult; % m (Down)
gps_vel_noise_std = 0.1 * gps_noise_mult; % m/s

sigma_a_proc  = 0.2;                     % m/s^2 (white accel driving v)
sigma_w_proc  = deg2rad(0.1);            % rad/s (white rate driving theta)

% Bias random-walk spectral densities (tune as needed)
sigma_ba_proc = 0.005;                   % m/s^2/sqrt(s) (accel bias RW)
sigma_bg_proc = deg2rad(0.02);           % rad/s/sqrt(s) (gyro bias RW)

%% EKF state: x = [p_n, p_d, v_n, v_d, theta, b_ax, b_az, b_g]'
nx = 8;
x_hat = zeros(nx,1);
P     = eye(nx) * 10.0;

%% Process noise Q (discrete)
Q = zeros(nx);
Q_pv = [dt^4/4, dt^3/2; dt^3/2, dt^2] * (sigma_a_proc^2);
Q([1 3],[1 3]) = Q_pv;   % North: [p_n, v_n]
Q([2 4],[2 4]) = Q_pv;   % Down:  [p_d, v_d]
Q(5,5) = (dt * sigma_w_proc)^2;     % theta driven by white rate

% Bias random walk: b_{k+1} = b_k + w*dt  => var increment (dt*sigma)^2
Q(6,6) = (dt * sigma_ba_proc)^2;    % b_ax
Q(7,7) = (dt * sigma_ba_proc)^2;    % b_az
Q(8,8) = (dt * sigma_bg_proc)^2;    % b_g

%% Measurement model: z = [p_n, p_d, v_n, v_d]
H = [1 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 1 0 0 0 0];

R = diag([gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2]);

%% Logs
x_hist = zeros(N,nx);
P_hist = zeros(N,nx);
nis_hist = nan(N,1);
nees_hist = nan(N,1);
I = eye(nx);

%% EKF loop
for k = 1:N
    % Unpack
    p_n = x_hat(1); 
    p_d = x_hat(2);
    v_n = x_hat(3); 
    v_d = x_hat(4);
    theta = x_hat(5);
    b_ax  = x_hat(6);
    b_az  = x_hat(7);
    b_g   = x_hat(8);

    % Precompute
    ct = cos(theta); st = sin(theta);

    % Bias-corrected IMU
    fx = fx_meas(k) - b_ax;
    fz = fz_meas(k) - b_az;
    qw = qw_meas(k) - b_g;

    % Specific force to nav accelerations (pitch-only ND)
    % a_n = -C_nb f_b + g_n  (using user's sign convention)
    aN = -(fx*ct + fz*st);         % North
    aD =  (fx*st - fz*ct + g);     % Down

    % Predict
    x_hat(1) = p_n + v_n * dt;
    x_hat(2) = p_d + v_d * dt;
    x_hat(3) = v_n + aN * dt;
    x_hat(4) = v_d + aD * dt;
    x_hat(5) = theta + qw * dt;

    % Bias random walk (constant in mean)
    x_hat(6) = b_ax;
    x_hat(7) = b_az;
    x_hat(8) = b_g;

    % Jacobian F
    F = I;
    F(1,3) = dt; 
    F(2,4) = dt;
    % ∂v_N/∂θ and ∂v_D/∂θ (via aN,aD)
    F(3,5) = dt * ( fx*st - fz*ct );
    F(4,5) = dt * ( fx*ct + fz*st );
    % ∂v/∂b_ax and ∂v/∂b_az (since fx=fmx-b_ax, fz=fmz-b_az)
    F(3,6) = dt * (  ct );   % ∂aN/∂b_ax
    F(3,7) = dt * (  st );   % ∂aN/∂b_az
    F(4,6) = dt * ( -st );   % ∂aD/∂b_ax
    F(4,7) = dt * (  ct );   % ∂aD/∂b_az
    % ∂theta/∂b_g
    F(5,8) = -dt;

    % Covariance predict
    P = F * P * F' + Q;

    % Update (when GPS N,D pos/vel available)
    if ~isnan(gps_pos_n_data(k)) && ~isnan(gps_pos_d_data(k)) && ...
       ~isnan(gps_vel_n_data(k)) && ~isnan(gps_vel_d_data(k))

        z = [gps_pos_n_data(k);
             gps_pos_d_data(k);
             gps_vel_n_data(k);
             gps_vel_d_data(k)];

        y = z - H * x_hat;
        S = H * P * H' + R;
        K = P * H' / S;
        x_hat = x_hat + K * y;

        % Joseph form for covariance update
        A = (I - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P'); % symmetry

        % NIS (Normalized Innovation Squared)
        nis_hist(k) = y' / S * y;

        % NEES (Normalized Estimation Error Squared)
        x_true = [ref_pos_n_data(k); ref_pos_d_data(k); ref_vel_n_data(k); ref_vel_d_data(k); ref_theta(k); 0; 0; 0];
        err = x_hat - x_true;
        nees_hist(k) = err' / P * err;
    end

    % Log
    x_hist(k,:) = x_hat';
    P_hist(k,:) = diag(P)';
end

%% Extract states
est_p_n = x_hist(:,1); est_p_d = x_hist(:,2);
est_v_n = x_hist(:,3); est_v_d = x_hist(:,4);
est_th  = x_hist(:,5);
est_bax = x_hist(:,6); est_baz = x_hist(:,7); est_bg = x_hist(:,8);

%% Plots
lw = 1.5;
idx_pn = ~isnan(gps_pos_n_data);
idx_pd = ~isnan(gps_pos_d_data);
idx_vn = ~isnan(gps_vel_n_data);
idx_vd = ~isnan(gps_vel_d_data);

% Position North
figure('Name','Position North'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_pos_n_data, 'k-', 'LineWidth', lw, 'DisplayName','ref p_N');
plot(t, est_p_n,        'b-', 'LineWidth', lw, 'DisplayName','ekf p_N');
scatter(t(idx_pn), gps_pos_n_data(idx_pn), 18, 'r', 'filled', 'DisplayName','GPS p_N');
legend('Location','best'); xlabel t; ylabel('p_N [m]');
nexttile; hold on; grid on;
plot(t, est_p_n - ref_pos_n_data, 'b-', 'LineWidth', lw);
scatter(t(idx_pn), gps_pos_n_data(idx_pn) - ref_pos_n_data(idx_pn), 18, 'r', 'filled');
xlabel t; ylabel('error [m]');

% Position Down
figure('Name','Position Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_pos_d_data, 'k-', 'LineWidth', lw, 'DisplayName', 'ref p_D');
plot(t, est_p_d,        'b-', 'LineWidth', lw, 'DisplayName', 'ekf p_D');
scatter(t(idx_pd), gps_pos_d_data(idx_pd), 18, 'r', 'filled', 'DisplayName','GPS p_D');
legend('Location','best'); xlabel t; ylabel('p_D [m]');
nexttile; hold on; grid on;
plot(t, est_p_d - ref_pos_d_data, 'b-', 'LineWidth', lw);
scatter(t(idx_pd), gps_pos_d_data(idx_pd) - ref_pos_d_data(idx_pd), 18, 'r', 'filled');
xlabel t; ylabel('error [m]');

% Velocity North
figure('Name','Velocity North'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_n_data, 'k-', 'LineWidth', lw, 'DisplayName','ref v_N');
plot(t, est_v_n,        'b-', 'LineWidth', lw, 'DisplayName','ekf v_N');
scatter(t(idx_vn), gps_vel_n_data(idx_vn), 18, 'r', 'filled', 'DisplayName','GPS v_N');
legend('Location','best'); xlabel t; ylabel('v_N [m/s]');
nexttile; hold on; grid on;
plot(t, est_v_n - ref_vel_n_data, 'b-', 'LineWidth', lw);
scatter(t(idx_vn), gps_vel_n_data(idx_vn) - ref_vel_n_data(idx_vn), 18, 'r', 'filled');
xlabel t; ylabel('error [m/s]');

% Velocity Down
figure('Name','Velocity Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_d_data, 'k-', 'LineWidth', lw, 'DisplayName','ref v_D');
plot(t, est_v_d,        'b-', 'LineWidth', lw, 'DisplayName','ekf v_D');
scatter(t(idx_vd), gps_vel_d_data(idx_vd), 18, 'r', 'filled', 'DisplayName','GPS v_D');
legend('Location','best'); xlabel t; ylabel('v_D [m/s]');
nexttile; hold on; grid on;
plot(t, est_v_d - ref_vel_d_data, 'b-', 'LineWidth', lw);
scatter(t(idx_vd), gps_vel_d_data(idx_vd) - ref_vel_d_data(idx_vd), 18, 'r', 'filled');
xlabel t; ylabel('error [m/s]');

% Theta (Pitch)
figure('Name','Theta'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, rad2deg(ref_theta), 'k-', 'LineWidth', lw, 'DisplayName','ref \theta [deg]');
plot(t, rad2deg(est_th),    'b-', 'LineWidth', lw, 'DisplayName','ekf \theta [deg]');
legend('Location','best'); xlabel t; ylabel('\theta [deg]');
nexttile; hold on; grid on;
theta_err_rad = atan2(sin(est_th - ref_theta), cos(est_th - ref_theta));
plot(t, rad2deg(theta_err_rad), 'm-', 'LineWidth', lw);
xlabel t; ylabel('error [deg]');

% Biases
figure('Name','Bias States'); tiledlayout(3,1);
nexttile; hold on; grid on;
plot(t, est_bax, 'b-', 'LineWidth', lw); ylabel('b_{ax} [m/s^2]'); xlabel t;
nexttile; hold on; grid on;
plot(t, est_baz, 'b-', 'LineWidth', lw); ylabel('b_{az} [m/s^2]'); xlabel t;
nexttile; hold on; grid on;
plot(t, rad2deg(est_bg), 'b-', 'LineWidth', lw); ylabel('b_g [deg/s]'); xlabel t;

%% NIS and NEES Analysis
meas_dim = 4;          % [p_n, p_d, v_n, v_d]
state_dim = nx;        % 8

alpha = 0.05;
nis_ci_lower = chi2inv(alpha / 2, meas_dim);
nis_ci_upper = chi2inv(1 - alpha / 2, meas_dim);
nees_ci_lower = chi2inv(alpha / 2, state_dim);
nees_ci_upper = chi2inv(1 - alpha / 2, state_dim);

figure('Name', 'NIS Analysis');
hold on; grid on;
plot(t, nis_hist, 'b.', 'MarkerSize', 8, 'DisplayName', 'NIS');
plot(t, ones(size(t)) * nis_ci_lower, 'r--', 'LineWidth', lw, 'DisplayName', '95% CI Lower');
plot(t, ones(size(t)) * nis_ci_upper, 'r--', 'LineWidth', lw, 'DisplayName', '95% CI Upper');
title('Normalized Innovation Squared (NIS)');
xlabel('Time (s)'); ylabel('NIS'); legend('show');

figure('Name', 'NEES Analysis');
hold on; grid on;
plot(t, nees_hist, 'b.', 'MarkerSize', 8, 'DisplayName', 'NEES');
plot(t, ones(size(t)) * nees_ci_lower, 'r--', 'LineWidth', lw, 'DisplayName', '95% CI Lower');
plot(t, ones(size(t)) * nees_ci_upper, 'r--', 'LineWidth', lw, 'DisplayName', '95% CI Upper');
title('Normalized Estimation Error Squared (NEES)');
xlabel('Time (s)'); ylabel('NEES'); legend('show');

avg_nis = nanmean(nis_hist);
avg_nees = nanmean(nees_hist);
fprintf('\n--- Filter Consistency Analysis ---\n');
fprintf('Measurement Dimension (for NIS): %d\n', meas_dim);
fprintf('Average NIS: %.4f (should be close to %d)\n', avg_nis, meas_dim);
fprintf('State Dimension (for NEES): %d\n', state_dim);
fprintf('Average NEES: %.4f (should be close to %d)\n', avg_nees, state_dim);
fprintf('-----------------------------------\n');
