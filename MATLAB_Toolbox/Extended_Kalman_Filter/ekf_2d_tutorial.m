% EKF (specific force input) in North–Down; no bias states
clc; clear; close all;

%% Run Simulink if needed
 dt_gps = 1/5;
 dt_ekf = 1/1000;
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
ref_pos_e_data  = ref_pos_data(:, 2);
ref_pos_d_data  = ref_pos_data(:, 3);
ref_vel_data    = squeeze(Sim.ref_Ve.Data);    % [N E D]
ref_vel_n_data  = ref_vel_data(:, 1);
ref_vel_e_data  = ref_vel_data(:, 2);
ref_vel_d_data  = ref_vel_data(:, 3);
ref_euler_data  = squeeze(Sim.ref_euler.Data); % [roll pitch yaw]
ref_phi         = ref_euler_data(:, 1);        % roll
ref_theta       = ref_euler_data(:, 2);        % pitch
ref_psi         = ref_euler_data(:, 3);        % yaw

%% Sensors
% Accelerometer outputs specific force in body: f_b = [fx fy fz] with f_b = g - a_actual
accel_data = squeeze(Sim.accel.Data)';   % [fx fy fz] specific force (body)
fx_meas = accel_data(:,1);               % body X specific force
fy_meas = accel_data(:,2);               % body Y specific force
fz_meas = accel_data(:,3);               % body Z specific force

gyro_data  = squeeze(Sim.gyro.Data)';    % [p q r] (body)
pw_meas = gyro_data(:,1);                % body roll rate
qw_meas = gyro_data(:,2);                % body pitch rate
rw_meas = gyro_data(:,3);                % body yaw rate

% GPS aligned to IMU rate with NaNs when absent
gps_pos_data    = squeeze(Sim.gps_Xe.Data)';    % [N E D]
gps_pos_n_data  = gps_pos_data(:, 1);
gps_pos_e_data  = gps_pos_data(:, 2);
gps_pos_d_data  = gps_pos_data(:, 3);

gps_vel_data    = squeeze(Sim.gps_Ve.Data)';   % [N E D]
gps_vel_n_data  = gps_vel_data(:, 1);
gps_vel_e_data  = gps_vel_data(:, 2);
gps_vel_d_data  = gps_vel_data(:, 3);


%% Noise settings
gps_pos_noise_std = 2.5; % m (North)
gps_alt_noise_std = 3.0; % m (Down)
gps_vel_noise_std = 0.1; % m/s
sigma_a_proc  = 0.2;                    % m/s^2
sigma_w_proc  = deg2rad(0.1);           % rad/s

%% EKF State: x = [p_n, p_d, v_n, v_d, theta]'
x_hat = zeros(5, 1);
P = eye(5) * 1.0; % Initial uncertainty, no coupling and 1 unit uncertainty for each axis.

% Q matrix = covariance matrix
% The varience of each state due to sensor noise
% Change in V due to accelerometer noise:
% delta_v = a_rand * dt
% We need VAR(delta_v) for our Q matrix
% VAR(delta_v) = VAR(a_rand * dt)
% var_v = dt^2 * VAR(a_rand) = std_a^2 * dt^2
var_v = sigma_a_proc^2 * dt^2;

% delta_p = 1/2 * dt^2 * a_rand
% var_p = (1/2)^2 * (dt^2)^2 * std_a^2
var_p = (1/4) * sigma_a_proc^2 * dt^4;

% The covariance, coupling between V and P
% Cov(delta_p, delta_v) = E((delta_p) x (delta_v))
% Cov(delta_p, delta_v) = E((1/2 * dt^2 * a_rand) x (a_rand * dt))
% Cov(delta_p, delta_v) = E((1/2) * a_rand^2 * dt^3)
% Cov(delta_p, delta_v) = 1/2 * dt^3 * E(a_rand^2)
% E(a_rand^2) = Var(a_rand) = std_a^2
cov_pv = (1/2) * sigma_a_proc^2 * dt^3;

% delta_theta = w_rand * dt
% var_theta = w_rand^2 * dt^2
var_theta = sigma_w_proc^2 * dt^2;

% Assuming each axes noise is the same
% x = [p_n, p_d, v_n, v_d, theta]'
Q = [   var_p       0       cov_pv   0           0;
        0           var_p   0           cov_pv   0
        cov_pv      0       var_v       0        0
        0           cov_pv  0           var_v    0
        0           0       0           0        var_theta];

% R matrix = correction 'trust' matrix
% Square matrix the size of our absolute measurement states
% GPS measurement states: vN, vD, pN, pD
% R matrix is also a varience matrix
% z = [p_n, p_d, v_n, v_d]
R = diag([gps_pos_noise_std^2 gps_alt_noise_std^2 gps_vel_noise_std^2 ...
        gps_vel_noise_std^2]);

% H matrix is the bridge between our measurement matrix and x_hat
H = [   1 0 0 0 0;
        0 1 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0];
% Logs
x_hist = zeros(N, 5);
P_hist = zeros(N, 5);
I5 = eye(5);

%% EKF Loop
for k = 1:N

    % Unpack
    p_n = x_hat(1);
    p_d = x_hat(2);
    v_n = x_hat(3);
    v_d = x_hat(4);
    theta = x_hat(5);

    % Precompute
    ct = cos(theta);
    st = sin(theta);
    
    % Specific force (raw accelerometer measurements
    fx = fx_meas(k);
    fz = fz_meas(k);

    % Convert specific force to body accelerations
    ax = -fx - g*st;
    az = -fz + g*ct;

    % Predicted accelerations in N and D
    aN = ax*ct + az*st;
    aD = az*ct - ax*st;

    % Gyro (no bias)
    qw = qw_meas(k);

    % Prediction step
    x_hat(1) = p_n + v_n * dt;
    x_hat(2) = p_d + v_d * dt;
    x_hat(3) = v_n + aN * dt;
    x_hat(4) = v_d + aD * dt;
    x_hat(5) = theta + qw * dt;

    % Jacobian matrix:
    % Linear approximation of our nonlinear equations
    % Starting with equations x_hat(1) = p_n + v_n * dt;
    % Take partial derivative of this equation with respect to each state
    % d/dp_n: p_n + v_n * dt = 1
    % d/dp_d: p_n + v_n * dt = 0
    % d/dv_n: p_n + v_n * dt = dt
    % d/dv_d: p_n + v_n * dt = 0
    % d/dtheta: p_d + v_d * dt = 0
    % Resulting row 1 of F = [1 0 dt 0 0]
    % Repeat for each row

    % d/d_theta: aN = ax*ct + az*st = az*ct - ax*st
    % d/d_theta: aD = az*ct - ax*st = -az*ct - ax*ct

    F = eye(5); % Start with an identity matrix, it's easier
    
    % Add the linear terms
    F(1,3) = dt;
    F(2,4) = dt;
    
    % Add the nonlinear terms from our correct derivatives
    % Remember to multiply by dt!
    F(3,5) = dt * (fx*st - fz*ct); % This is dt * (∂aN/∂theta)
    F(4,5) = dt * (fx*ct + fz*st); % This is dt * (∂aD/∂theta)

    
    P = F * P * F' + Q;

    % Update (when GPS is available)
    if ~isnan(gps_vel_d_data(k))
        z = [gps_pos_n_data(k);
             gps_pos_d_data(k);
             gps_vel_n_data(k);
             gps_vel_d_data(k)];

        y = z - H * x_hat;
        S = H * P * H' + R;
        K = P * H' / S;
        x_hat = x_hat + K * y;
        % Joseph Form
        A = (I5 - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P'); % Symmetry
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
