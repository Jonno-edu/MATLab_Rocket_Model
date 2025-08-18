% --- Build unified CSV at IMU rate with GNSS NED epochs marked + async MAG (µT) ---
clc; clear; close all;

% Run Simulink only if needed
if ~exist('Sim', 'var') || isempty(Sim)
    Sim = sim("simulate_sensors.slx");
end

% Helper to build timetables
buildTT = @(t, X, names) timetable(seconds(t), X(:,1), X(:,2), X(:,3), 'VariableNames', names);

% --- Sensor signals (native rates) ---
t_gps  = Sim.gps_Xe.Time;   X_gps  = parse(Sim.gps_Xe.Data, 3);   % N,E,D (m)
t_gv   = Sim.gps_Ve.Time;   V_gps  = parse(Sim.gps_Ve.Data, 3);   % V_N,V_E,V_D (m/s)
t_acc  = Sim.accel.Time;    A_body = parse(Sim.accel.Data, 3);    % aX,aY,aZ (m/s^2)
t_gyro = Sim.gyro.Time;     G_body = parse(Sim.gyro.Data, 3);     % p,q,r (rad/s)
t_mag  = Sim.mag.Time;      M_body = parse(Sim.mag.Data, 3);      % mX,mY,mZ (microtesla)

TT_gps_Xe  = buildTT(t_gps,  X_gps,  {'N','E','D'});
TT_gps_Ve  = buildTT(t_gv,   V_gps,  {'V_N','V_E','V_D'});
TT_accel   = buildTT(t_acc,  A_body, {'aX','aY','aZ'});
TT_gyro    = buildTT(t_gyro, G_body, {'p','q','r'});
TT_mag     = buildTT(t_mag,  M_body, {'mX_uT','mY_uT','mZ_uT'});

% --- Define IMU timeline (use accel timebase) ---
t0     = TT_accel.Time(1);
t1     = TT_accel.Time(end);
dt_acc = median(seconds(diff(TT_accel.Time)));
imu_hz = round(1/dt_acc);
fprintf('Inferred IMU rate: %d Hz\n', imu_hz);

imu_time_vec = (t0:seconds(1/imu_hz):t1)';     % datetime array
TT_imu_time  = timetable(imu_time_vec, zeros(numel(imu_time_vec),1), 'VariableNames', {'dummy'});
imu_times = imu_time_vec;

% --- Resample accel/gyro to IMU timeline (nearest sample) ---
TT_acc_rs  = retime(TT_accel, imu_times, 'nearest');
TT_gyro_rs = retime(TT_gyro,  imu_times, 'nearest');

% --- Map GNSS and MAG epochs to nearest IMU tick (with guards) ---
N = numel(imu_times);

% MAG slots (asynchronous to IMU)
mag_valid  = false(N,1);
mX_uT      = nan(N,1);
mY_uT      = nan(N,1);
mZ_uT      = nan(N,1);

% GNSS slots (asynchronous to IMU)
gnss_valid = false(N,1);
N_m        = nan(N,1);
E_m        = nan(N,1);
D_m        = nan(N,1);
V_N_mps    = nan(N,1);
V_E_mps    = nan(N,1);
V_D_mps    = nan(N,1);
iTOW_ms    = nan(N,1);

% Time bases relative to t0
imu_t_s = seconds(imu_times - t0);
mag_t_s = seconds(TT_mag.Time - t0);
gps_t_s = seconds(TT_gps_Xe.Time - t0);

% --- Policy: "first-hit wins" to avoid clobbering same IMU tick ---

% GNSS -> nearest IMU tick (first-hit wins)
for k = 1:numel(gps_t_s)
    [~, idx] = min(abs(imu_t_s - gps_t_s(k)));
    if ~gnss_valid(idx)
        gnss_valid(idx) = true;
        N_m(idx)      = TT_gps_Xe.N(k);
        E_m(idx)      = TT_gps_Xe.E(k);
        D_m(idx)      = TT_gps_Xe.D(k);
        V_N_mps(idx)  = TT_gps_Ve.V_N(k);
        V_E_mps(idx)  = TT_gps_Ve.V_E(k);
        V_D_mps(idx)  = TT_gps_Ve.V_D(k);
        iTOW_ms(idx)  = round(seconds(TT_gps_Xe.Time(k) - TT_gps_Xe.Time(1)) * 1000);
    end
end

% MAG -> nearest IMU tick (first-hit wins)
for k = 1:numel(mag_t_s)
    [~, idx] = min(abs(imu_t_s - mag_t_s(k)));
    if ~mag_valid(idx)
        mag_valid(idx) = true;
        mX_uT(idx) = TT_mag.mX_uT(k);
        mY_uT(idx) = TT_mag.mY_uT(k);
        mZ_uT(idx) = TT_mag.mZ_uT(k);
    end
end

% --- Optional diagnostics ---
fprintf('Mapped MAG samples to %d/%d IMU ticks\n', nnz(mag_valid), N);
fprintf('Mapped GNSS samples to %d/%d IMU ticks\n', nnz(gnss_valid), N);

% --- Verify lengths before table build (catch mismatches early) ---
t_us = uint64(round(imu_t_s * 1e6));  % microseconds since start

assert(numel(t_us) == N, 't_us length %d ~= N %d', numel(t_us), N);
assert(height(TT_acc_rs)  == N, 'TT_acc_rs rows %d ~= N %d',  height(TT_acc_rs),  N);
assert(height(TT_gyro_rs) == N, 'TT_gyro_rs rows %d ~= N %d', height(TT_gyro_rs), N);

assert(numel(mX_uT)      == N, 'mX_uT length %d ~= N %d',      numel(mX_uT),      N);
assert(numel(mY_uT)      == N, 'mY_uT length %d ~= N %d',      numel(mY_uT),      N);
assert(numel(mZ_uT)      == N, 'mZ_uT length %d ~= N %d',      numel(mZ_uT),      N);
assert(numel(mag_valid)  == N, 'mag_valid length %d ~= N %d',  numel(mag_valid),  N);

assert(numel(gnss_valid) == N, 'gnss_valid length %d ~= N %d', numel(gnss_valid), N);
assert(numel(N_m)        == N, 'N_m length %d ~= N %d',        numel(N_m),        N);
assert(numel(E_m)        == N, 'E_m length %d ~= N %d',        numel(E_m),        N);
assert(numel(D_m)        == N, 'D_m length %d ~= N %d',        numel(D_m),        N);
assert(numel(V_N_mps)    == N, 'V_N_mps length %d ~= N %d',    numel(V_N_mps),    N);
assert(numel(V_E_mps)    == N, 'V_E_mps length %d ~= N %d',    numel(V_E_mps),    N);
assert(numel(V_D_mps)    == N, 'V_D_mps length %d ~= N %d',    numel(V_D_mps),    N);
assert(numel(iTOW_ms)    == N, 'iTOW_ms length %d ~= N %d',    numel(iTOW_ms),    N);

% --- Construct export table (USE ONLY N-length arrays; do not use TT_mag.* here) ---
Sensors = table( ...
    t_us, ...
    TT_acc_rs.aX, TT_acc_rs.aY, TT_acc_rs.aZ, ...
    TT_gyro_rs.p, TT_gyro_rs.q, TT_gyro_rs.r, ...
    mX_uT, mY_uT, mZ_uT, mag_valid, ...
    gnss_valid, ...
    N_m, E_m, D_m, ...
    V_N_mps, V_E_mps, V_D_mps, ...
    iTOW_ms, ...
    'VariableNames', { ...
        't_us', ...
        'ax','ay','az', ...
        'gx','gy','gz', ...
        'mX_uT','mY_uT','mZ_uT','mag_valid', ...
        'gnss_valid', ...
        'N_m','E_m','D_m', ...
        'V_N_mps','V_E_mps','V_D_mps', ...
        'iTOW_ms' ...
    } ...
);

% --- Write CSV ---
% outDir = fullfile('tools','data');
% if ~exist(outDir, 'dir'); mkdir(outDir); end
% outFile = fullfile(outDir, 'sim_run.csv');
% writetable(T, outFile);
% fprintf('Wrote %s with %d rows at %d Hz IMU timeline (GNSS+MAG async mapped, µT).\n', outFile, height(T), imu_hz);
