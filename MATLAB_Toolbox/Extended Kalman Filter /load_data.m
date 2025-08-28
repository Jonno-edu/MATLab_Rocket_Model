% EKF 2 rewritten to North–Down with EKF 1 logic/maths — plotting fixed
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

%% Sensors (map Simulink -> EKF conventions)
accel_data = squeeze(Sim.accel.Data)';   % [ax ay az] from Simulink
gyro_data  = squeeze(Sim.gyro.Data)';    % [p q r]    from Simulink

% Use measured total acceleration directly; EKF needs specific force: f_b = -a_meas (see loop)
ax_meas = accel_data(:,1);   % body X total acceleration
az_meas = accel_data(:,3);   % body Z total acceleration
qw_meas = -gyro_data(:,2);    % body pitch rate (flip later if needed)

% GPS aligned to IMU rate with NaNs when absent
gps_pos_data    = squeeze(Sim.gps_Xe.Data);    % [N E D]
gps_pos_n_data  = gps_pos_data(:, 1);
gps_pos_d_data  = gps_pos_data(:, 3);

gps_vel_data    = squeeze(Sim.gps_Ve.Data)';   % [N E D]
gps_vel_n_data  = gps_vel_data(:, 1);
gps_vel_d_data  = gps_vel_data(:, 3);

%% 4) GNSS nominal noise (for R)
gps_pos_noise_std = 2.5; % m (North)
gps_alt_noise_std = 3.0; % m (Down)
gps_vel_noise_std = 0.1; % m/s

%% 5) EKF (EKF1 logic; state: [p_n, p_d, v_n, v_d, theta, b_w, b_ax, b_az]')
x_hat = zeros(8,1);
P     = eye(8) * 1.0;
P(7,7) = (0.5)^2;
P(8,8) = (0.5)^2;

% Process/measurement noise (EKF1 style)
sigma_a_proc  = 0.2;                    % m/s^2
sigma_w_proc  = deg2rad(0.1);           % rad/s
sigma_bw_proc = deg2rad(0.001);         % rad/s^2
sigma_ba_proc = 0.005;                  % m/s^3

Q = zeros(8);
Q_pv = [dt^4/4, dt^3/2; dt^3/2, dt^2] * (sigma_a_proc^2); % discrete white-accel
Q([1 3],[1 3]) = Q_pv;  % North: [p_n, v_n]
Q([2 4],[2 4]) = Q_pv;  % Down:  [p_d, v_d]
Q(5,5) = (dt * sigma_w_proc )^2;
Q(6,6) = (dt * sigma_bw_proc)^2;
Q(7,7) = (dt * sigma_ba_proc)^2;
Q(8,8) = (dt * sigma_ba_proc)^2;

H = [eye(4), zeros(4,4)];
R = diag([gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2]);

% Logs
x_hist = zeros(N,8);
P_hist = zeros(N,8);
I8 = eye(8);

%% Loop
for k = 1:N
    % Unpack
    p_n = x_hat(1); 
    p_d = x_hat(2);
    v_n = x_hat(3); 
    v_d = x_hat(4);
    theta = x_hat(5);
    b_w = x_hat(6); 
    b_ax = x_hat(7); 
    b_az = x_hat(8);

    % Precompute
    ct = cos(theta); st = sin(theta);

    % Convert Simulink total acceleration (a_meas) to specific force: f_b = -a_meas
    fbx = -ax_meas(k);
    fbz = -az_meas(k);

    % Bias-corrected specific force and gyro
    axb = fbx - b_ax;
    azb = fbz - b_az;
    qw  = qw_meas(k) - b_w;
    % Prediction
    x_hat(1) = p_n + v_n * dt;
    x_hat(2) = p_d + v_d * dt;
    x_hat(3) = v_n + (axb*ct - azb*st) * dt;
    x_hat(4) = v_d + (axb*st + azb*ct + g) * dt;
    x_hat(5) = theta + qw * dt;
    % Bias states are random walks

    % Jacobian F
    F = I8;
    F(1,3) = dt; F(2,4) = dt;
    F(3,5) = (-axb*st - azb*ct) * dt;
    F(4,5) = ( axb*ct - azb*st) * dt;
    F(3,7) = -ct * dt;  F(3,8) =  st * dt;
    F(4,7) = -st * dt;  F(4,8) = -ct * dt;
    F(5,6) = -dt;

    % Covariance predict
    P = F * P * F' + Q;

    % Measurement update only if all 4 GPS components available
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

        % Joseph-form covariance update
        A = (I8 - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P'); % enforce symmetry
    end

    % Log
    x_hist(k,:) = x_hat';
    P_hist(k,:) = diag(P)';
end

%% 6) Extract states
est_p_n = x_hist(:,1); est_p_d = x_hist(:,2);
est_v_n = x_hist(:,3); est_v_d = x_hist(:,4);
est_th  = -x_hist(:,5);
b_w_est  = x_hist(:,6);
b_ax_est = x_hist(:,7);
b_az_est = x_hist(:,8);

%% 7) Plots
lw = 1.5; ms = 18;

% GPS availability indices
idx_pn = ~isnan(gps_pos_n_data);
idx_pd = ~isnan(gps_pos_d_data);
idx_vn = ~isnan(gps_vel_n_data);
idx_vd = ~isnan(gps_vel_d_data);

% -- Position North
figure('Name','Position North'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_pos_n_data, 'k-', 'LineWidth', lw, 'DisplayName','ref p_N');
plot(t, est_p_n,        'b-', 'LineWidth', lw, 'DisplayName','ekf p_N');
scatter(t(idx_pn), gps_pos_n_data(idx_pn), 18, 'r', 'filled', 'DisplayName','GPS p_N');
xlabel('t [s]'); ylabel('p_N [m]'); title('Position North'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, est_p_n - ref_pos_n_data, 'b-', 'LineWidth', lw, 'DisplayName','ekf - ref');
scatter(t(idx_pn), gps_pos_n_data(idx_pn) - ref_pos_n_data(idx_pn), 18, 'r', 'filled', 'DisplayName','gps - ref');
xlabel('t [s]'); ylabel('error [m]'); title('Position North Error'); legend('Location','best');

% -- Position Down
figure('Name','Position Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_pos_d_data, 'k-', 'LineWidth', lw, 'DisplayName','ref p_D');
plot(t, est_p_d,        'b-', 'LineWidth', lw, 'DisplayName','ekf p_D');
scatter(t(idx_pd), gps_pos_d_data(idx_pd), 18, 'r', 'filled', 'DisplayName','GPS p_D');
xlabel('t [s]'); ylabel('p_D [m]'); title('Position Down'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, est_p_d - ref_pos_d_data, 'b-', 'LineWidth', lw, 'DisplayName','ekf - ref');
scatter(t(idx_pd), gps_pos_d_data(idx_pd) - ref_pos_d_data(idx_pd), 18, 'r', 'filled', 'DisplayName','gps - ref');
xlabel('t [s]'); ylabel('error [m]'); title('Position Down Error'); legend('Location','best');

% -- Velocity North
figure('Name','Velocity North'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_n_data, 'k-', 'LineWidth', lw, 'DisplayName','ref v_N');
plot(t, est_v_n,        'b-', 'LineWidth', lw, 'DisplayName','ekf v_N');
scatter(t(idx_vn), gps_vel_n_data(idx_vn), 18, 'r', 'filled', 'DisplayName','GPS v_N');
xlabel('t [s]'); ylabel('v_N [m/s]'); title('Velocity North'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, est_v_n - ref_vel_n_data, 'b-', 'LineWidth', lw, 'DisplayName','ekf - ref');
scatter(t(idx_vn), gps_vel_n_data(idx_vn) - ref_vel_n_data(idx_vn), 18, 'r', 'filled', 'DisplayName','gps - ref');
xlabel('t [s]'); ylabel('error [m/s]'); title('Velocity North Error'); legend('Location','best');

% -- Velocity Down
figure('Name','Velocity Down'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, ref_vel_d_data, 'k-', 'LineWidth', lw, 'DisplayName','ref v_D');
plot(t, est_v_d,        'b-', 'LineWidth', lw, 'DisplayName','ekf v_D');
scatter(t(idx_vd), gps_vel_d_data(idx_vd), 18, 'r', 'filled', 'DisplayName','GPS v_D');
xlabel('t [s]'); ylabel('v_D [m/s]'); title('Velocity Down'); legend('Location','best');

nexttile; hold on; grid on;
plot(t, est_v_d - ref_vel_d_data, 'b-', 'LineWidth', lw, 'DisplayName','ekf - ref');
scatter(t(idx_vd), gps_vel_d_data(idx_vd) - ref_vel_d_data(idx_vd), 18, 'r', 'filled', 'DisplayName','gps - ref');
xlabel('t [s]'); ylabel('error [m/s]'); title('Velocity Down Error'); legend('Location','best');

% -- Pitch angle (degrees, wrapped error in degrees)
figure('Name','Theta'); tiledlayout(2,1);

% Top: reference vs EKF in degrees
nexttile; hold on; grid on;
plot(t, rad2deg(ref_theta), 'k-', 'LineWidth', lw, 'DisplayName','ref \theta [deg]');
plot(t, rad2deg(est_th),    'b-', 'LineWidth', lw, 'DisplayName','ekf \theta [deg]');
xlabel('t [s]'); ylabel('\theta [deg]'); title('Theta (degrees)'); legend('Location','best');

% Bottom: wrapped error in degrees
nexttile; hold on; grid on;
theta_err_rad = atan2(sin(est_th - ref_theta), cos(est_th - ref_theta)); % wrap to [-pi, pi]
plot(t, rad2deg(theta_err_rad), 'm-', 'LineWidth', lw, 'DisplayName','wrapped error [deg]');
xlabel('t [s]'); ylabel('error [deg]'); title('Theta Error (wrapped, degrees)'); legend('Location','best');

% -- Bias estimates (each in its own subplot)
figure('Name','Bias States'); 
tiledlayout(3,1);

% Gyro bias
nexttile; hold on; grid on;
plot(t, b_w_est, 'LineWidth', lw, 'DisplayName','b_w [rad/s]');
if exist('gyro_bias_true','var')
    yline(gyro_bias_true, '--', 'Color',[0.6 0 0], 'DisplayName','gyro bias true');
end
xlabel('t [s]'); ylabel('rad/s'); title('Gyro Bias b_w'); legend('Location','best');

% Accel X bias
nexttile; hold on; grid on;
plot(t, b_ax_est, 'LineWidth', lw, 'DisplayName','b_{ax} [m/s^2]');
if exist('accel_bias_true','var')
    yline(accel_bias_true, '--', 'Color',[0 0.4 0], 'DisplayName','accel bias true');
end
xlabel('t [s]'); ylabel('m/s^2'); title('Accel Bias b_{ax}'); legend('Location','best');

% Accel Z bias
nexttile; hold on; grid on;
plot(t, b_az_est, 'LineWidth', lw, 'DisplayName','b_{az} [m/s^2]');
if exist('accel_bias_true','var')
    yline(accel_bias_true, '--', 'Color',[0 0.4 0], 'DisplayName','accel bias true');
end
xlabel('t [s]'); ylabel('m/s^2'); title('Accel Bias b_{az}'); legend('Location','best');
