clc; close all;

% Files
measFile = fullfile('tools','data','sim_run.csv');
ekfFile  = '/Users/jonno/GitProjects/rocket-fc/build/debug/ekf_out.csv';  % adjust if needed

% Load measurement CSV
Tm = readtable(measFile);
t_m = double(Tm.t_us) * 1e-6;    % seconds from start

% Load EKF CSV (assumes header: t_s,pN,pE,pD,vN,vE,vD,roll_deg,pitch_deg,yaw_deg)
Te = readtable(ekfFile);
t_e = double(Te{:,1});           % t_s
pN_e = Te{:,2}; pE_e = Te{:,3}; pD_e = Te{:,4};
vN_e = Te{:,5}; vE_e = Te{:,6}; vD_e = Te{:,7};
roll_e  = Te{:,8};  % deg
pitch_e = Te{:,9};  % deg
yaw_e   = Te{:,10}; % deg

% Masks for GNSS validity in measurement log
posMask = ~isnan(Tm.N_m) & ~isnan(Tm.E_m) & ~isnan(Tm.D_m);
velMask = ~isnan(Tm.V_N_mps) & ~isnan(Tm.V_E_mps) & ~isnan(Tm.V_D_mps);

% Convenience handles
toDeg = @(x) x * 180/pi;

% 1) Attitude: EKF roll/pitch/yaw (deg) vs tilt/mag-derived rough angles from measurements
%    If you don't have measured attitude, we derive tilt-only from accel for a qualitative check.
hasAttMeas = false; % set true if a measured attitude exists elsewhere

if ~hasAttMeas
    % Compute tilt from accel (rough, only when norm~g)
    ax = Tm.ax; ay = Tm.ay; az = Tm.az;
    an = sqrt(ax.^2 + ay.^2 + az.^2);
    g  = 9.80665;
    tiltMask = abs(an - g) < 2.5; % heuristic

    roll_m  = atan2(ay, az);
    pitch_m = atan2(-ax, sqrt(ay.^2 + az.^2));

    figure('Name','Attitude (EKF vs tilt-from-accel)');
    subplot(3,1,1);
    plot(t_e, roll_e, 'b-', 'LineWidth',1.2); hold on;
    plot(t_m(tiltMask), toDeg(roll_m(tiltMask)), 'k.', 'MarkerSize',6);
    grid on; ylabel('Roll [deg]'); legend('EKF','Accel tilt~g','Location','best');
    title('Attitude comparison');

    subplot(3,1,2);
    plot(t_e, pitch_e, 'b-', 'LineWidth',1.2); hold on;
    plot(t_m(tiltMask), toDeg(pitch_m(tiltMask)), 'k.', 'MarkerSize',6);
    grid on; ylabel('Pitch [deg]'); legend('EKF','Accel tilt~g','Location','best');

    subplot(3,1,3);
    plot(t_e, yaw_e, 'b-', 'LineWidth',1.2);
    grid on; xlabel('Time [s]'); ylabel('Yaw [deg]'); legend('EKF','Location','best');
else
    % If you have measured attitude, put that plotting here
end

% 2) Position NED: EKF vs GNSS (scatter at valid epochs)
if any(posMask)
    figure('Name','Position NED: EKF vs GNSS');
    subplot(3,1,1);
    plot(t_e, pN_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(posMask), Tm.N_m(posMask), 8, 'k', 'filled');
    grid on; ylabel('N [m]'); legend('EKF','GNSS','Location','best'); title('Position comparison');

    subplot(3,1,2);
    plot(t_e, pE_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(posMask), Tm.E_m(posMask), 8, 'k', 'filled');
    grid on; ylabel('E [m]'); legend('EKF','GNSS','Location','best');

    subplot(3,1,3);
    plot(t_e, pD_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(posMask), Tm.D_m(posMask), 8, 'k', 'filled');
    grid on; ylabel('D [m]'); xlabel('Time [s]'); legend('EKF','GNSS','Location','best');
end

% 3) Velocity NED: EKF vs GNSS (scatter at valid epochs)
if any(velMask)
    figure('Name','Velocity NED: EKF vs GNSS');
    subplot(3,1,1);
    plot(t_e, vN_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(velMask), Tm.V_N_mps(velMask), 8, 'k', 'filled');
    grid on; ylabel('V_N [m/s]'); legend('EKF','GNSS','Location','best'); title('Velocity comparison');

    subplot(3,1,2);
    plot(t_e, vE_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(velMask), Tm.V_E_mps(velMask), 8, 'k', 'filled');
    grid on; ylabel('V_E [m/s]'); legend('EKF','GNSS','Location','best');

    subplot(3,1,3);
    plot(t_e, vD_e, 'b-', 'LineWidth',1.2); hold on;
    scatter(t_m(velMask), Tm.V_D_mps(velMask), 8, 'k', 'filled');
    grid on; ylabel('V_D [m/s]'); xlabel('Time [s]'); legend('EKF','GNSS','Location','best');
end

% 4) Optional residual plots (GNSS)
if any(posMask)
    % Interpolate EKF to GNSS time for residuals
    pN_e_i = interp1(t_e, pN_e, t_m(posMask), 'linear', 'extrap');
    pE_e_i = interp1(t_e, pE_e, t_m(posMask), 'linear', 'extrap');
    pD_e_i = interp1(t_e, pD_e, t_m(posMask), 'linear', 'extrap');

    rN = Tm.N_m(posMask) - pN_e_i;
    rE = Tm.E_m(posMask) - pE_e_i;
    rD = Tm.D_m(posMask) - pD_e_i;

    figure('Name','Position Residuals (GNSS - EKF)');
    subplot(3,1,1); plot(t_m(posMask), rN, 'k.'); grid on; ylabel('dN [m]'); title('Residuals');
    subplot(3,1,2); plot(t_m(posMask), rE, 'k.'); grid on; ylabel('dE [m]');
    subplot(3,1,3); plot(t_m(posMask), rD, 'k.'); grid on; ylabel('dD [m]'); xlabel('Time [s]');
end

if any(velMask)
    vN_e_i = interp1(t_e, vN_e, t_m(velMask), 'linear', 'extrap');
    vE_e_i = interp1(t_e, vE_e, t_m(velMask), 'linear', 'extrap');
    vD_e_i = interp1(t_e, vD_e, t_m(velMask), 'linear', 'extrap');

    rVn = Tm.V_N_mps(velMask) - vN_e_i;
    rVe = Tm.V_E_mps(velMask) - vE_e_i;
    rVd = Tm.V_D_mps(velMask) - vD_e_i;

    figure('Name','Velocity Residuals (GNSS - EKF)');
    subplot(3,1,1); plot(t_m(velMask), rVn, 'k.'); grid on; ylabel('dV_N [m/s]'); title('Residuals');
    subplot(3,1,2); plot(t_m(velMask), rVe, 'k.'); grid on; ylabel('dV_E [m/s]');
    subplot(3,1,3); plot(t_m(velMask), rVd, 'k.'); grid on; ylabel('dV_D [m/s]'); xlabel('Time [s]');
end

% 5) Quick sanity: show first few EKF rows for verification
disp('First 5 EKF rows:');
disp(Te(1:min(5,height(Te)), :));
