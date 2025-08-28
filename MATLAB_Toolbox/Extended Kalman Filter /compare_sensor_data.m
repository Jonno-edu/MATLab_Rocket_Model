% Plot Simulink reference vs sensor + MATLAB-generated IMU comparison (North–Down)
clc; clear; close all;

%% Run Simulink
Sim = sim("simulate_sensors.slx");
g = 9.81;

%% Fetch Simulink sensors (body IMU) and references
t_acc  = Sim.accel.Time;
accel_b = -1 .* squeeze(Sim.accel.Data)';     % [ax ay az]
t_gyro = Sim.gyro.Time;
gyro_b  = squeeze(Sim.gyro.Data)';      % [p q r]

ref_t_eul = Sim.ref_euler.Time;
ref_eul   = squeeze(Sim.ref_euler.Data); % [roll pitch yaw]
ref_t_Ve  = Sim.ref_Ve.Time;
ref_Ve    = squeeze(Sim.ref_Ve.Data);    % [N E D]
ref_t_Xe  = Sim.ref_Xe.Time;
ref_Xe    = squeeze(Sim.ref_Xe.Data);    % [N E D]

%% Generate constant body specific force [5 0 -30] m/s² with 0 pitch rate
% Use Simulink timebase for easy overlay
t = t_acc(:);
N = length(t);

% Bias and noise settings
noise_multiplier = 1;
bias_multiplier  = 0;
gyro_noise_std   = deg2rad(0.5*noise_multiplier);
gyro_bias_true   = deg2rad(0.2*bias_multiplier);   % rad/s
accel_noise_std  = 0.05*noise_multiplier;          % m/s^2
accel_bias_true  = 0.02*bias_multiplier;           % m/s^2

% Generated IMU: constant body specific force [5 0 -30], zero pitch rate
perfect_accel_x_gen = 5 * ones(N, 1);     % constant +5 m/s² in body x
perfect_accel_z_gen = -30 * ones(N, 1);   % constant -30 m/s² in body z
perfect_gyro_th_gen = zeros(N, 1);        % zero pitch rate

% Add noise and bias to generated IMU
accel_x_gen = perfect_accel_x_gen + randn(N,1)*accel_noise_std + accel_bias_true;
accel_z_gen = perfect_accel_z_gen + randn(N,1)*accel_noise_std + accel_bias_true;
gyro_q_gen  = perfect_gyro_th_gen + randn(N,1)*gyro_noise_std  + gyro_bias_true;

%% Reference on accelerometer timebase (for Simulink IMU overlay)
ref_theta = interp1(ref_t_eul, ref_eul(:,2), t, 'linear', 'extrap');   % pitch [rad]
ref_vn    = interp1(ref_t_Ve,  ref_Ve(:,1),   t, 'linear', 'extrap');  % N vel
ref_vd    = interp1(ref_t_Ve,  ref_Ve(:,3),   t, 'linear', 'extrap');  % D vel
a_n = gradient(ref_vn, t);                                             % N accel
a_d = gradient(ref_vd, t);                                             % D accel

% Reference body specific force from Simulink nav accel (for comparison to Simulink IMU accel)
ct = cos(ref_theta); st = sin(ref_theta);
f_bx_ref =  a_n.*ct + a_d.*st - g.*st;   % from nav to body specific force
f_bz_ref = -a_n.*st + a_d.*ct - g.*ct;

% Sensor pitch rate aligned to accel timebase; reference pitch rate
q_imu_sim = interp1(t_gyro, gyro_b(:,2), t, 'linear', 'extrap');       % Simulink sensor q
q_ref     = gradient(ref_theta, t);                                    % Simulink ref thetadot

%% GPS streams (positions and velocities)
t_gps_pos = Sim.gps_Xe.Time;
gps_pos   = squeeze(Sim.gps_Xe.Data);    % [N E D]
t_gps_vel = Sim.gps_Ve.Time;
gps_vel   = squeeze(Sim.gps_Ve.Data)';   % [N E D]

% Interpolate references to GPS time bases for clean overlays
ref_pn_on_gps = interp1(ref_t_Xe, ref_Xe(:,1), t_gps_pos, 'linear', 'extrap');
ref_pd_on_gps = interp1(ref_t_Xe, ref_Xe(:,3), t_gps_pos, 'linear', 'extrap');
ref_vn_on_gps = interp1(ref_t_Ve, ref_Ve(:,1), t_gps_vel, 'linear', 'extrap');
ref_vd_on_gps = interp1(ref_t_Ve, ref_Ve(:,3), t_gps_vel, 'linear', 'extrap');

%% Plots

% Body accelerations: Simulink reference vs Simulink sensor vs Generated IMU
lw = 1.6;
figure('Name','Body Accelerations Comparison'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t, f_bx_ref, 'k-', 'LineWidth', lw, 'DisplayName','Simulink ref f_bx');
plot(t, accel_b(:,1), 'b--', 'LineWidth', 1.2, 'DisplayName','Simulink sensor a_x');
plot(t, perfect_accel_x_gen, 'r:', 'LineWidth', lw, 'DisplayName','Gen perfect a_x (5)');
plot(t, accel_x_gen, 'm:', 'LineWidth', 1.2, 'DisplayName','Gen noisy a_x');
xlabel('t [s]'); ylabel('m/s^2'); title('Body X Specific Force: Simulink vs Generated'); 
legend('Location','best');

nexttile; hold on; grid on;
plot(t, f_bz_ref, 'k-', 'LineWidth', lw, 'DisplayName','Simulink ref f_bz');
plot(t, accel_b(:,3), 'b--', 'LineWidth', 1.2, 'DisplayName','Simulink sensor a_z');
plot(t, perfect_accel_z_gen, 'r:', 'LineWidth', lw, 'DisplayName','Gen perfect a_z (-30)');
plot(t, accel_z_gen, 'm:', 'LineWidth', 1.2, 'DisplayName','Gen noisy a_z');
xlabel('t [s]'); ylabel('m/s^2'); title('Body Z Specific Force: Simulink vs Generated'); 
legend('Location','best');

% Pitch rate: Simulink reference vs Simulink sensor vs Generated IMU
figure('Name','Pitch Rate Comparison'); hold on; grid on;
plot(t, q_ref, 'k-', 'LineWidth', lw, 'DisplayName','Simulink ref \thetȧ');
plot(t, q_imu_sim, 'b--', 'LineWidth', 1.2, 'DisplayName','Simulink sensor q');
plot(t, perfect_gyro_th_gen, 'r:', 'LineWidth', lw, 'DisplayName','Gen perfect q (0)');
plot(t, gyro_q_gen, 'm:', 'LineWidth', 1.2, 'DisplayName','Gen noisy q');
xlabel('t [s]'); ylabel('rad/s'); title('Pitch Rate: Simulink vs Generated'); 
legend('Location','best');

% GPS Position (North & Down) - Simulink only
figure('Name','GPS Position (North–Down)'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t_gps_pos, ref_pn_on_gps, 'k-', 'LineWidth', lw, 'DisplayName','ref p_N');
scatter(t_gps_pos, gps_pos(:,1), 18, 'r', 'filled', 'DisplayName','GPS p_N');
xlabel('t [s]'); ylabel('m'); title('Position North'); legend('Location','best');

nexttile; hold on; grid on;
plot(t_gps_pos, ref_pd_on_gps, 'k-', 'LineWidth', lw, 'DisplayName','ref p_D');
scatter(t_gps_pos, gps_pos(:,3), 18, 'r', 'filled', 'DisplayName','GPS p_D');
xlabel('t [s]'); ylabel('m'); title('Position Down'); legend('Location','best');

% GPS Velocity (North & Down) - Simulink only
figure('Name','GPS Velocity (North–Down)'); tiledlayout(2,1);
nexttile; hold on; grid on;
plot(t_gps_vel, ref_vn_on_gps, 'k-', 'LineWidth', lw, 'DisplayName','ref v_N');
scatter(t_gps_vel, gps_vel(:,1), 18, 'r', 'filled', 'DisplayName','GPS v_N');
xlabel('t [s]'); ylabel('m/s'); title('Velocity North'); legend('Location','best');

nexttile; hold on; grid on;
plot(t_gps_vel, ref_vd_on_gps, 'k-', 'LineWidth', lw, 'DisplayName','ref v_D');
scatter(t_gps_vel, gps_vel(:,3), 18, 'r', 'filled', 'DisplayName','GPS v_D');
xlabel('t [s]'); ylabel('m/s'); title('Velocity Down'); legend('Location','best');

%% Display generated IMU stats for verification
fprintf('Generated IMU Statistics:\n');
fprintf('  Body ax: perfect = %g m/s², mean noisy = %.3f m/s²\n', 5, mean(accel_x_gen));
fprintf('  Body az: perfect = %g m/s², mean noisy = %.3f m/s²\n', -30, mean(accel_z_gen));
fprintf('  Pitch rate: perfect = %g rad/s, mean noisy = %.4f rad/s\n', 0, mean(gyro_q_gen));
fprintf('  True biases: accel = %.4f m/s², gyro = %.4f rad/s\n', accel_bias_true, gyro_bias_true);
