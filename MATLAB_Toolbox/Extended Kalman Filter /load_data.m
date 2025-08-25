% --- Build unified CSV at IMU rate with GNSS NED epochs marked + async MAG (µT) ---
clc; clear; close all;

% === Run Simulink only if needed ===
if ~exist('Sim', 'var') || isempty(Sim)
    Sim = sim("simulate_sensors.slx");
end

% === Small helpers ===
parse3 = @(ts) parse(ts.Data, 3);                       % parse 3-col timeseries.Data
buildTT = @(t, X, names) timetable(seconds(t), X(:,1), X(:,2), X(:,3), ...
                                   'VariableNames', names);


% 1. Squeeze your data to get a 3xN matrix
g_B_perfect_squeezed = squeeze(Sim.g_B_meas.Data);

% 2. Define the original and target number of points
original_len = size(g_B_perfect_squeezed, 2); % Should be 300001
target_len = 60001; % The desired length of your gyro data

% 3. Create the original and target 'x' coordinates for interpolation
%    These are just normalized indices from 1 to N
x_original = linspace(1, original_len, original_len);
x_target = linspace(1, original_len, target_len);

% 4. Interpolate each of the three axes (rows) to the new length
g_B_perfect_resampled = interp1(x_original, g_B_perfect_squeezed', x_target)';

% 5. Verify the new size
disp('Original size:');
disp(size(g_B_perfect_squeezed));
disp('Resampled size:');
disp(size(g_B_perfect_resampled));

% === Sensor signals (native rates) ===
t_gps  = Sim.gps_Xe.Time;    X_gps  = parse3(Sim.gps_Xe);     % N,E,D (m)
t_gv   = Sim.gps_Ve.Time;    V_gps  = parse3(Sim.gps_Ve);     % V_N,V_E,V_D (m/s)
t_acc  = Sim.accel.Time;     A_body = parse3(Sim.accel);      % aX,aY,aZ (m/s^2)
t_gyro = Sim.gyro.Time;      G_body = parse3(Sim.gyro);       % p,q,r (rad/s)
t_mag  = Sim.mag.Time;       M_body = parse3(Sim.mag);        % mX,mY,mZ (µT)
t_B_perf = Sim.g_B_meas.Time; G_perf = parse3(Sim.g_B_meas);


TT_gps_Xe = buildTT(t_gps,  X_gps,  {'N','E','D'});
TT_gps_Ve = buildTT(t_gv,   V_gps,  {'V_N','V_E','V_D'});
TT_accel  = buildTT(t_acc,  A_body, {'aX','aY','aZ'});
TT_gyro   = buildTT(t_gyro, G_body, {'p','q','r'});
TT_mag    = buildTT(t_mag,  M_body, {'mX_uT','mY_uT','mZ_uT'});
TT_B_perf = buildTT(t_B_perf, G_perf, {'GX_perf', 'GY_perf', 'GZ_perf'});

% === Define IMU timeline (use accel timebase) ===
t0      = TT_accel.Time(1);
t1      = TT_accel.Time(end);
dt_acc  = median(seconds(diff(TT_accel.Time)));
imu_hz  = round(1/dt_acc);
fprintf('Inferred IMU rate: %d Hz\n', imu_hz);

imu_time_vec = (t0:seconds(1/imu_hz):t1)';     % datetime array
TT_imu_time  = timetable(imu_time_vec, zeros(numel(imu_time_vec),1), 'VariableNames', {'dummy'});
imu_times    = imu_time_vec;

% === Resample accel/gyro to IMU timeline (nearest sample) ===
TT_acc_rs  = retime(TT_accel, imu_times, 'nearest');
TT_gyro_rs = retime(TT_gyro,  imu_times, 'nearest');

% === Map GNSS and MAG epochs to nearest IMU tick (first-hit wins) ===
N = numel(imu_times);

% MAG slots (asynchronous to IMU)
mag_valid = false(N,1);
mX_uT     = nan(N,1);
mY_uT     = nan(N,1);
mZ_uT     = nan(N,1);

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

% --- GNSS -> nearest IMU tick (first-hit wins)
for k = 1:numel(gps_t_s)
    [~, idx] = min(abs(imu_t_s - gps_t_s(k)));
    if ~gnss_valid(idx)
        gnss_valid(idx) = true;
        N_m(idx)     = TT_gps_Xe.N(k);
        E_m(idx)     = TT_gps_Xe.E(k);
        D_m(idx)     = TT_gps_Xe.D(k);
        V_N_mps(idx) = TT_gps_Ve.V_N(k);
        V_E_mps(idx) = TT_gps_Ve.V_E(k);
        V_D_mps(idx) = TT_gps_Ve.V_D(k);
        
        iTOW_ms(idx) = round(seconds(TT_gps_Xe.Time(k) - TT_gps_Xe.Time(1)) * 1000);
    end
end

% --- MAG -> nearest IMU tick (first-hit wins)
for k = 1:numel(mag_t_s)
    [~, idx] = min(abs(imu_t_s - mag_t_s(k)));
    if ~mag_valid(idx)
        mag_valid(idx) = true;
        mX_uT(idx) = TT_mag.mX_uT(k);
        mY_uT(idx) = TT_mag.mY_uT(k);
        mZ_uT(idx) = TT_mag.mZ_uT(k);
        
    end
end

% === Optional diagnostics ===
fprintf('Mapped MAG samples to %d/%d IMU ticks\n', nnz(mag_valid), N);
fprintf('Mapped GNSS samples to %d/%d IMU ticks\n', nnz(gnss_valid), N);

% === Verify lengths before table build (catch mismatches early) ===
t_us = uint64(round(imu_t_s * 1e6));  % microseconds since start

assert(numel(t_us)      == N);
assert(height(TT_acc_rs)== N);
assert(height(TT_gyro_rs)==N);
assert(numel(mX_uT)     == N && numel(mY_uT) == N && numel(mZ_uT) == N);
assert(numel(mag_valid) == N);
assert(numel(gnss_valid)== N);
assert(numel(N_m)       == N && numel(E_m)   == N && numel(D_m)   == N);
assert(numel(V_N_mps)   == N && numel(V_E_mps)==N && numel(V_D_mps)==N);
assert(numel(iTOW_ms)   == N);

% === Construct export table (use only N-length arrays) ===
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
        'p','q','r', ...
        'mX_uT','mY_uT','mZ_uT','mag_valid', ...
        'gnss_valid', ...
        'N_m','E_m','D_m', ...
        'V_N_mps','V_E_mps','V_D_mps', ...
        'iTOW_ms' ...
    } ...
);

% =======================
% === Parse ref_ signals
% =======================

% ref_Xe: N,E,D (m)
t_ref_Xe  = Sim.ref_Xe.Time;   X_ref_Xe  = parse3(Sim.ref_Xe);
TT_ref_Xe = buildTT(t_ref_Xe,  X_ref_Xe, {'N_ref','E_ref','D_ref'});

% ref_Ve: V_N,V_E,V_D (m/s)
t_ref_Ve  = Sim.ref_Ve.Time;   X_ref_Ve  = parse3(Sim.ref_Ve);
TT_ref_Ve = buildTT(t_ref_Ve,  X_ref_Ve, {'V_N_ref','V_E_ref','V_D_ref'});

% ref_Ab: body accelerations (m/s^2)
t_ref_Ab  = Sim.ref_Ab.Time;   X_ref_Ab  = parse3(Sim.ref_Ab);
TT_ref_Ab = buildTT(t_ref_Ab,  X_ref_Ab, {'aX_b_ref','aY_b_ref','aZ_b_ref'});

% ref_Ae: earth-frame accelerations (m/s^2)
t_ref_Ae  = Sim.ref_Ae.Time;   X_ref_Ae  = parse3(Sim.ref_Ae);
TT_ref_Ae = buildTT(t_ref_Ae,  X_ref_Ae, {'aN_e_ref','aE_e_ref','aD_e_ref'});

% ref_attitude: phi, theta, psi (rad)
t_ref_att  = Sim.ref_attitude.Time;   X_ref_att  = parse3(Sim.ref_attitude);
TT_ref_att = buildTT(t_ref_att,       X_ref_att, {'phi_ref','theta_ref','psi_ref'});

% ref_pqr: body rates (rad/s)
t_ref_pqr  = Sim.ref_pqr.Time;   X_ref_pqr  = parse3(Sim.ref_pqr);
TT_ref_pqr = buildTT(t_ref_pqr,  X_ref_pqr, {'p_ref','q_ref','r_ref'});

% ===============================
% === Resample refs to IMU ticks
% ===============================
TT_ref_Xe_rs  = retime(TT_ref_Xe,  imu_times, 'nearest');
TT_ref_Ve_rs  = retime(TT_ref_Ve,  imu_times, 'nearest');
TT_ref_Ab_rs  = retime(TT_ref_Ab,  imu_times, 'nearest');
TT_ref_Ae_rs  = retime(TT_ref_Ae,  imu_times, 'nearest');
TT_ref_att_rs = retime(TT_ref_att, imu_times, 'nearest');
TT_ref_pqr_rs = retime(TT_ref_pqr, imu_times, 'nearest');

% === Sanity checks ===
assert(height(TT_ref_Xe_rs)  == N);
assert(height(TT_ref_Ve_rs)  == N);
assert(height(TT_ref_Ab_rs)  == N);
assert(height(TT_ref_Ae_rs)  == N);
assert(height(TT_ref_att_rs) == N);
assert(height(TT_ref_pqr_rs) == N);

% ==========================================
% === Extend export table with ref_ signals
% ==========================================
Sensors = [ ...
    Sensors, table( ...
        TT_ref_Xe_rs.N_ref, TT_ref_Xe_rs.E_ref, TT_ref_Xe_rs.D_ref, ...
        TT_ref_Ve_rs.V_N_ref, TT_ref_Ve_rs.V_E_ref, TT_ref_Ve_rs.V_D_ref, ...
        TT_ref_Ab_rs.aX_b_ref, TT_ref_Ab_rs.aY_b_ref, TT_ref_Ab_rs.aZ_b_ref, ...
        TT_ref_Ae_rs.aN_e_ref, TT_ref_Ae_rs.aE_e_ref, TT_ref_Ae_rs.aD_e_ref, ...
        TT_ref_att_rs.phi_ref, TT_ref_att_rs.theta_ref, TT_ref_att_rs.psi_ref, ...
        TT_ref_pqr_rs.p_ref, TT_ref_pqr_rs.q_ref, TT_ref_pqr_rs.r_ref, ...
        'VariableNames', { ...
            'N_ref','E_ref','D_ref', ...
            'V_N_ref','V_E_ref','V_D_ref', ...
            'aX_b_ref','aY_b_ref','aZ_b_ref', ...
            'aN_e_ref','aE_e_ref','aD_e_ref', ...
            'phi_ref','theta_ref','psi_ref', ...
            'p_ref','q_ref','r_ref' ...
        } ...
    ) ...
];

