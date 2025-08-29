% EKF (specific force input) in North–Down; no bias states
clc; clear; close all;

%% Run Simulink if needed
if ~exist('Sim', 'var') || isempty(Sim)
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
% Accelerometer outputs specific force in body: f_b = [fx fy fz] with f_b = g - a_actual
accel_data = squeeze(Sim.accel.Data)';   % [fx fy fz] specific force (body)
gyro_data  = squeeze(Sim.gyro.Data)';    % [p q r] (body)
fx_meas = accel_data(:,1);               % body X specific force
fz_meas = accel_data(:,3);               % body Z specific force
qw_meas = gyro_data(:,2);                % body pitch rate

% GPS aligned to IMU rate with NaNs when absent
gps_pos_data    = squeeze(Sim.gps_Xe.Data);    % [N E D]
gps_pos_n_data  = gps_pos_data(:, 1);
gps_pos_d_data  = gps_pos_data(:, 3);
gps_vel_data    = squeeze(Sim.gps_Ve.Data)';   % [N E D]
gps_vel_n_data  = gps_vel_data(:, 1);
gps_vel_d_data  = gps_vel_data(:, 3);

%% Noise settings
gps_pos_noise_std = 2.5; % m (North)
gps_alt_noise_std = 3.0; % m (Down)
gps_vel_noise_std = 0.1; % m/s
sigma_a_proc  = 0.2;                    % m/s^2
sigma_w_proc  = deg2rad(0.1);           % rad/s

%% EKF state: x = [p_n, p_d, v_n, v_d, theta]'
x_hat = zeros(5,1);
P     = eye(5) * 1.0;

% Process noise Q (white-accel on v, white-rate on theta)
Q = zeros(5);
Q_pv = [dt^4/4, dt^3/2; dt^3/2, dt^2] * (sigma_a_proc^2);
Q([1 3],[1 3]) = Q_pv;   % North: [p_n, v_n]
Q([2 4],[2 4]) = Q_pv;   % Down:  [p_d, v_d]
Q(5,5) = (dt * sigma_w_proc)^2;

% Measurement model: z = [p_n, p_d, v_n, v_d]
H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 1 0];
R = diag([gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2]);

% Logs
x_hist = zeros(N,5);
P_hist = zeros(N,5);
I5 = eye(5);

%% EKF loop
for k = 1:N
    % Unpack
    p_n = x_hat(1); 
    p_d = x_hat(2);
    v_n = x_hat(3); 
    v_d = x_hat(4);
    theta = x_hat(5);

    % Precompute
    ct = cos(theta); st = sin(theta);

    % Specific force (bias-free). Use a = -C_nb*f_b + g_n for pitch-only N–D
    fx = fx_meas(k);
    fz = fz_meas(k);

    % Predicted accelerations in N and D
    aN = -(fx*ct + fz*st);        % North
    aD =  (fx*st - fz*ct + g);    % Down

    % Gyro (no bias state)
    qw = qw_meas(k);

    % Predict
    x_hat(1) = p_n + v_n * dt;
    x_hat(2) = p_d + v_d * dt;
    x_hat(3) = v_n + aN * dt;
    x_hat(4) = v_d + aD * dt;
    x_hat(5) = theta + qw * dt;

    % Jacobian F
    F = I5;
    F(1,3) = dt; 
    F(2,4) = dt;
    F(3,5) = dt * ( fx*st - fz*ct );   % ∂v_N/∂θ
    F(4,5) = dt * ( fx*ct + fz*st );   % ∂v_D/∂θ

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
        % Joseph form
        A = (I5 - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P'); % symmetry
    end

    % Log
    x_hist(k,:) = x_hat';
    P_hist(k,:) = diag(P)';
end

%% Extract states
est_p_n = x_hist(:,1); est_p_d = x_hist(:,2);
est_v_n = x_hist(:,3); est_v_d = x_hist(:,4);
est_th  = x_hist(:,5);

%% Plots
lw = 1.5;
idx_pn = ~isnan(gps_pos_n_data);
idx_pd = ~isnan(gps_pos_d_data);
idx_vn = ~isnan(gps_vel_n_data);
idx_vd = ~isnan(gps_vel_d_data);

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

figure('Name','Position Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_pos_d_data, 'k-', 'LineWidth', lw);
plot(t, est_p_d,        'b-', 'LineWidth', lw);
scatter(t(idx_pd), gps_pos_d_data(idx_pd), 18, 'r', 'filled');
legend('Location','best'); xlabel t; ylabel('p_D [m]');
nexttile; hold on; grid on;
plot(t, est_p_d - ref_pos_d_data, 'b-', 'LineWidth', lw);
scatter(t(idx_pd), gps_pos_d_data(idx_pd) - ref_pos_d_data(idx_pd), 18, 'r', 'filled');
xlabel t; ylabel('error [m]');

figure('Name','Velocity North'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_n_data, 'k-', 'LineWidth', lw);
plot(t, est_v_n,        'b-', 'LineWidth', lw);
scatter(t(idx_vn), gps_vel_n_data(idx_vn), 18, 'r', 'filled');
legend('Location','best'); xlabel t; ylabel('v_N [m/s]');
nexttile; hold on; grid on;
plot(t, est_v_n - ref_vel_n_data, 'b-', 'LineWidth', lw);
scatter(t(idx_vn), gps_vel_n_data(idx_vn) - ref_vel_n_data(idx_vn), 18, 'r', 'filled');
xlabel t; ylabel('error [m/s]');

figure('Name','Velocity Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_d_data, 'k-', 'LineWidth', lw);
plot(t, est_v_d,        'b-', 'LineWidth', lw);
scatter(t(idx_vd), gps_vel_d_data(idx_vd), 18, 'r', 'filled');
legend('Location','best'); xlabel t; ylabel('v_D [m/s]');
nexttile; hold on; grid on;
plot(t, est_v_d - ref_vel_d_data, 'b-', 'LineWidth', lw);
scatter(t(idx_vd), gps_vel_d_data(idx_vd) - ref_vel_d_data(idx_vd), 18, 'r', 'filled');
xlabel t; ylabel('error [m/s]');

figure('Name','Theta'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, rad2deg(ref_theta), 'k-', 'LineWidth', lw, 'DisplayName','ref \theta [deg]');
plot(t, rad2deg(est_th),    'b-', 'LineWidth', lw, 'DisplayName','ekf \theta [deg]');
legend('Location','best'); xlabel t; ylabel('\theta [deg]');
nexttile; hold on; grid on;
theta_err_rad = atan2(sin(est_th - ref_theta), cos(est_th - ref_theta));
plot(t, rad2deg(theta_err_rad), 'm-', 'LineWidth', lw);
xlabel t; ylabel('error [deg]');
