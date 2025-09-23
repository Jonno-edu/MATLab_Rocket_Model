% IMEKF (trajectory‑independent H) with direction-only accel/mag
% Rocket-like launch accelerations after 100 s (in body frame, units of g)
% State: [0:2]=dtheta, [3:5]=dv, [6:8]=dp, [9:11]=bg, [12:14]=ba, [15:17]=bm
clear; clc;

%% Parameters
fs = 200; dt = 1/fs; N = 50000;                 % 250 s total
T_static = 100.0;                               % warm-up (stationary) until 100 s
g_i = [0;0;-1];                                 % gravity (unit g)
m_i = [1;0;0];                                  % magnetic field (unit)
ENABLE_DEWEIGHT = true;                         % toggle accel de-weighting

% Sensor noise
sig_g = deg2rad(0.05);    % gyro white noise [rad/s]
sig_a = 0.02;             % accel direction noise [unit]
sig_m = 0.01;             % mag direction noise [unit]

% Gyro bias: constant + random walk
bg_const = deg2rad([0.05, 0.05, 0.05]);   % constant bias [rad/s]
q_bg     = (deg2rad(0.02))^2;             % RW spectral density

% Accel/mag bias random walks (direction model)
q_ba = (0.0005)^2;
q_bm = (0.0005)^2;

% Process noise (drivers)
q_gyro_noise  = (sig_g^2)*eye(3);
q_accel_noise = (sig_a^2)*eye(3);

% Measurement covariance (direction-only)
R_acc_base = (sig_a^2)*eye(3);
R_mag      = (sig_m^2)*eye(3);

% Time
t  = (0:N-1)'*dt;

%% Truth attitude (static)
w_true = zeros(N,3);
q_true = zeros(N,4); q_true(1,:) = [1 0 0 0];
for k=1:N-1
    q_true(k+1,:) = q_true(k,:);
end

% Ideal body-frame vectors
a_b_true = zeros(N,3); m_b_true = zeros(N,3);
for k=1:N
    Rbw = quat_to_R_b2w(q_true(k,:));   % body->world
    Rib = Rbw.';                        % world->body
    a_b_true(k,:) = (Rib*g_i).';
    m_b_true(k,:) = (Rib*m_i).';
end

%% Rocket-like body linear acceleration a_lin_b (units of g), t >= 100 s
a_lin_b = zeros(N,3);
t0 = T_static;
for k=1:N
    tr = t(k) - t0;  % time since liftoff start
    if tr <= 0, continue; end

    % Longitudinal (z-axis)
    if tr < 5
        aL = 0.3 + 0.1*(tr/5);
    elseif tr < 50
        aL = 0.4 + 2.1*((tr-5)/45);
    elseif tr < 70
        aL = 2.5 - 0.7*((tr-50)/20);
    elseif tr < 120
        aL = 1.8 + 1.7*((tr-70)/50);
    elseif tr < 125
        aL = 3.5 - 3.3*((tr-120)/5);
    elseif tr < 130
        aL = 0.2 + 1.0*((tr-125)/5);
    elseif tr < 200
        aL = 1.2 + 2.8*((tr-130)/70);
    else
        aL = 4.0;
    end

    % Lateral (x,y): gravity turn
    if tr < 10
        a_lat_x = 0;
        a_lat_y = 0;
    elseif tr < 30
        a_lat_x = 0.05 * ((tr-10)/20);
        a_lat_y = 0.2  * ((tr-10)/20);
    else
        a_lat_x = 0.5;  % steady lateral
        a_lat_y = 0.2;
    end

    % TVC dither (disabled here)
    a_tvc_x = 0.005*randn*0;
    a_tvc_y = 0.005*randn*0;

    a_lin_b(k,1) = a_lat_x + a_tvc_x;
    a_lin_b(k,2) = a_lat_y + a_tvc_y;
    a_lin_b(k,3) = aL;
end

%% Sensor simulation (white noise + bias: constant + random-walk)
bg_rw = zeros(1,3);
ba = zeros(1,3);
bm = zeros(1,3);

bg_true = zeros(N,3);
ba_true = zeros(N,3);
bm_true = zeros(N,3);

w_meas = zeros(N,3);
a_meas = zeros(N,3);
m_meas = zeros(N,3);

a_norm_raw = zeros(N,1); % pre-normalization magnitude for gating

for k=1:N
    % Gyro bias RW
    bg_rw = bg_rw + sqrt(q_bg*dt)*randn(1,3);
    ba    = ba    + sqrt(q_ba*dt)*randn(1,3);
    bm    = bm    + sqrt(q_bm*dt)*randn(1,3);

    bg_total = bg_const + bg_rw;
    bg_true(k,:) = bg_total; ba_true(k,:) = ba; bm_true(k,:) = bm;

    % Gyro measurement
    w_meas(k,:) = w_true(k,:) + bg_total + sig_g*randn(1,3);

    % Accelerometer specific force model: f_b = g_b - a_lin_b (direction-only)
    a_raw_vec = (a_b_true(k,:) - a_lin_b(k,:)) + ba + sig_a*randn(1,3);
    a_norm_raw(k) = norm(a_raw_vec);
    a_meas(k,:)   = a_raw_vec / max(a_norm_raw(k),1e-12);

    % Magnetometer (direction-only)
    m_raw_vec = m_b_true(k,:) + bm + sig_m*randn(1,3);
    m_meas(k,:) = m_raw_vec / max(norm(m_raw_vec),1e-12);
end

%% Warm-up alignment and bias seeding (pad)
idx0 = find(t <= T_static);

% Gyro bias seed
bg_est = mean(w_meas(idx0,:), 1);

% Reconstruct unnormalized accel from stored norm and direction
a_raw = a_meas(idx0,:) .* a_norm_raw(idx0);     % row-wise scale
a_raw_mean = mean(a_raw, 1);

% Roll/pitch from gravity direction (normalized)
a_dir = a_raw_mean / max(norm(a_raw_mean),1e-12);
roll0  = atan2( a_dir(2), a_dir(3) );
pitch0 = atan2(-a_dir(1), sqrt(a_dir(2)^2 + a_dir(3)^2) );

% Yaw from tilt-compensated magnetometer (use averaged direction)
m_dir = mean(m_meas(idx0,:), 1);
m_dir = m_dir / max(norm(m_dir),1e-12);
mx = m_dir(1); my = m_dir(2); mz = m_dir(3);
cr = cos(roll0);  sr = sin(roll0);
cp = cos(pitch0); sp = sin(pitch0);
mxh = mx*cp + mz*sp;
myh = mx*sr*sp + my*cr - mz*sr*cp;
yaw0 = atan2(-myh, mxh);

% Initial quaternion from ZYX yaw-pitch-roll
cy = cos(yaw0*0.5);  sy = sin(yaw0*0.5);
cP = cos(pitch0*0.5); sP = sin(pitch0*0.5);
cR = cos(roll0*0.5);  sR = sin(roll0*0.5);
q_est = [ cy*cP*cR + sy*sP*sR, ...
          cy*cP*sR - sy*sP*cR, ...
          cy*sP*cR + sy*cP*sR, ...
          sy*cP*cR - cy*sP*sR ];
q_est = quat_norm(q_est);

% Accel bias seed (using unnormalized mean)
Rbw0 = quat_to_R_b2w(q_est); Rib0 = Rbw0.';           % world->body
g_b0 = (Rib0*g_i).';                                   % expected gravity in body
ba_est = a_raw_mean - g_b0;

% Magnetometer bias seed (prefer preflight cal; keep zero here)
bm_est = [0 0 0];

% Slow accel/mag bias random walks during boost
q_ba = (1e-6)^2;
q_bm = (1e-6)^2;

%% IMEKF init
P = blkdiag(1e-2*eye(3), 1e-1*eye(3), 1e-1*eye(3), 1e-2*eye(3), 1e-2*eye(3), 1e-2*eye(3));
q_hat = zeros(N,4); q_hat(1,:) = q_est;
bg_hat = zeros(N,3); ba_hat = zeros(N,3); bm_hat = zeros(N,3);
bg_hat(1,:) = bg_est; ba_hat(1,:) = ba_est; bm_hat(1,:) = bm_est;

innov_log = zeros(N,6);

% De-weighting LPF state
scale_R_lpf = 1.0; tau_R = 0.3; alpha_R = exp(-dt/tau_R);
kappa = 50; deadband = 0.02;

%% IMEKF loop (trajectory‑independent measurement Jacobian)
for k=1:N-1
    % Adaptive accel measurement covariance + magnitude gating
    if ENABLE_DEWEIGHT
        delta_g = abs(a_norm_raw(k) - 1.0);                         % | ||a|| - 1 |
        gate_thresh = 0.2;                                          % reject if >0.2 g deviation
        if delta_g > gate_thresh
            R_acc = 1e6 * R_acc_base;                               % reject accel update
        else
            scale_raw = 1 + kappa*max(0, delta_g - deadband);       % >= 1
            scale_R_lpf = alpha_R*scale_R_lpf + (1-alpha_R)*scale_raw;
            R_acc = (scale_R_lpf^2) * R_acc_base;                   % variance scales with square
        end
    else
        R_acc = R_acc_base;
    end
    R = blkdiag(R_acc, R_mag);

    % Propagate (attitude + biases)
    omega = (w_meas(k,:) - bg_est);
    q_est = quat_norm( quat_mul(q_est, [0 0.5*omega*dt]) + q_est );

    % Error dynamics Jacobian
    G = zeros(18,18);
    G(1:3,1:3)   = -skew(omega);
    G(1:3,10:12) = -eye(3);

    Rbw = quat_to_R_b2w(q_est); Rib = Rbw.';
    a_body = (a_meas(k,:) - ba_est).';
    G(4:6,1:3)   = -Rib*skew(a_body);
    G(4:6,13:15) = -Rib;
    G(7:9,4:6)   = eye(3);

    F = eye(18) + G*dt;

    % Process noise Q
    Q = zeros(18,18);
    Q(1:3,1:3)       = q_gyro_noise*dt + q_bg*(dt^3)/3*eye(3);
    Q(1:3,10:12)     = -q_bg*(dt^2)/2*eye(3);
    Q(10:12,1:3)     = Q(1:3,10:12).';
    Q(10:12,10:12)   = q_bg*dt*eye(3);

    Q(4:6,4:6)       = q_accel_noise*dt + q_ba*(dt^3)/3*eye(3);
    Q(4:6,7:9)       = q_accel_noise*(dt^2)/2*eye(3) + q_ba*(dt^4)/8*eye(3);
    Q(7:9,4:6)       = Q(4:6,7:9).';
    Q(7:9,7:9)       = q_accel_noise*(dt^3)/3*eye(3) + q_ba*(dt^5)/20*eye(3);

    Q(4:6,13:15)     = -q_ba*(dt^2)/2*eye(3);
    Q(13:15,4:6)     = Q(4:6,13:15).';
    Q(7:9,13:15)     = -q_ba*(dt^3)/6*eye(3);
    Q(13:15,7:9)     = Q(7:9,13:15).';
    Q(13:15,13:15)   = q_ba*dt*eye(3);

    Q(16:18,16:18)   = q_bm*dt*eye(3);

    % Covariance propagate
    P = F*P*F.' + Q;

    % Predict unit vectors (for innovation only)
    a_pred = (Rib*g_i).';
    m_pred = (Rib*m_i).';

    % Invariant measurement Jacobian H (use measured body vectors)
    a_meas_b = a_meas(k,:).';
    m_meas_b = m_meas(k,:).';
    H = zeros(6,18);
    H(1:3,1:3) = skew(a_meas_b);
    H(4:6,1:3) = skew(m_meas_b);
    Pa = eye(3) - a_meas_b*a_meas_b.';
    Pm = eye(3) - m_meas_b*m_meas_b.';
    H(1:3,13:15) = Pa;
    H(4:6,16:18) = Pm;

    % Update (with optional accel gate)
    z    = [a_meas(k,:)  m_meas(k,:)].';
    zhat = [a_pred       m_pred     ].';
    S_full = H*P*H.' + R;
    innov = z - zhat;

    if ENABLE_DEWEIGHT
        nu_acc = innov(1:3).' / S_full(1:3,1:3) * innov(1:3);
        if nu_acc > 7.81, R(1:3,1:3) = 1e6 * R_acc_base; end
    end

    S = H*P*H.' + R;
    K = (P*H.')/S;
    dx = K*(z - zhat);

    % Retraction
    q_est = quat_norm( quat_mul(q_est, [1 0.5*dx(1:3).']) );
    bg_est = bg_est + dx(10:12).';
    ba_est = ba_est + dx(13:15).';
    bm_est = bm_est + dx(16:18).';

    % Joseph form
    I = eye(18);
    P = (I - K*H)*P*(I - K*H).' + K*R*K.';

    % Log
    q_hat(k+1,:) = q_est;
    bg_hat(k+1,:) = bg_est;
    ba_hat(k+1,:) = ba_est;
    bm_hat(k+1,:) = bm_est;
    innov_log(k+1,:) = (z - zhat).';
end

%% Euler angles
eul_true = zeros(N,3); eul_est = zeros(N,3);
for k=1:N
    eul_true(k,:) = quat_to_eulZYX(q_true(k,:));
    eul_est(k,:)  = quat_to_eulZYX(q_hat(k,:));
end
eul_true_deg = rad2deg(eul_true);
eul_est_deg  = rad2deg(eul_est);

%% Plots
% Euler
figure('Name','Euler Angles (ZYX): Truth vs IMEKF');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,eul_true_deg(:,3),'k-',t,eul_est_deg(:,3),'r--'); grid on; ylabel('Roll [deg]'); legend('True','IMEKF','Location','best'); xline(T_static,'k--','Warm-up end');
nexttile; plot(t,eul_true_deg(:,2),'k-',t,eul_est_deg(:,2),'r--'); grid on; ylabel('Pitch [deg]'); xline(T_static,'k--','Warm-up end');
nexttile; plot(t,eul_true_deg(:,1),'k-',t,eul_est_deg(:,1),'r--'); grid on; ylabel('Yaw [deg]'); xlabel('Time [s]'); xline(T_static,'k--','Warm-up end');

% Accelerations
f_b_mag = vecnorm(a_b_true - a_lin_b, 2, 2);
delta_g = abs(f_b_mag - 1.0);
figure('Name','Body Linear Accelerations (unit g)');
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(t, a_lin_b(:,3),'r-'); grid on; ylabel('a_z [g]'); xline(T_static,'k--','Warm-up end');
nexttile; plot(t, a_lin_b(:,1),'b-'); grid on; ylabel('a_x [g]'); xline(T_static,'k--','Warm-up end');
nexttile; plot(t, a_lin_b(:,2),'m-'); grid on; ylabel('a_y [g]'); xline(T_static,'k--','Warm-up end');
nexttile; plot(t, f_b_mag,'k-', t, delta_g,'r--'); grid on; ylabel('||f_b||, |·-1| [g]'); xlabel('Time [s]'); legend('||f_b||','Deviation','Location','best'); xline(T_static,'k--','Warm-up end');

% Gyro Bias
figure('Name','Gyro Bias (rad/s)');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for i=1:3
    nexttile; plot(t,bg_true(:,i),'k-',t,bg_hat(:,i),'r--'); grid on; ylabel(sprintf('b_g_%c','x'+i-1));
    if i==1, legend('True (const+RW)','Est','Location','best'); end
    xline(T_static,'k--','Warm-up end');
end
xlabel('Time [s]');

% Accel Bias
figure('Name','Accel Bias (unit)');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for i=1:3
    nexttile; plot(t,ba_true(:,i),'k-',t,ba_hat(:,i),'r--'); grid on; ylabel(sprintf('b_a_%c','x'+i-1));
    if i==1, legend('True','Est','Location','best'); end
    xline(T_static,'k--','Warm-up end');
end
xlabel('Time [s]');

% Mag Bias
figure('Name','Mag Bias (unit)');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for i=1:3
    nexttile; plot(t,bm_true(:,i),'k-',t,bm_hat(:,i),'r--'); grid on; ylabel(sprintf('b_m_%c','x'+i-1));
    if i==1, legend('True','Est','Location','best'); end
    xline(T_static,'k--','Warm-up end');
end
xlabel('Time [s]');

% Innovations
figure('Name','Innovation Norms');
plot(t, vecnorm(innov_log(:,1:3),2,2),'b-', t, vecnorm(innov_log(:,4:6),2,2),'m--'); grid on;
xlabel('Time [s]'); ylabel('||innov||'); legend('Accel','Mag','Location','best'); xline(T_static,'k--','Warm-up end');

%% Helpers
function S = skew(v)
S = [  0   -v(3)  v(2);
      v(3)   0   -v(1);
     -v(2)  v(1)   0  ];
end

function q = quat_norm(q)
q = q / max(norm(q),1e-12);
end

function q = quat_mul(q1, q2)
w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
q = [ w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
      w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
      w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
      w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end

function Rbw = quat_to_R_b2w(q) % body->world
w=q(1); x=q(2); y=q(3); z=q(4);
Rbw = [1-2*(y^2+z^2),   2*(x*y - z*w), 2*(x*z + y*w);
       2*(x*y + z*w), 1-2*(x^2+z^2),   2*(y*z - x*w);
       2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x^2+y^2)];
end

function eul = quat_to_eulZYX(q) % [yaw pitch roll]
w=q(1); x=q(2); y=q(3); z=q(4);
sy = 2*(w*z + x*y); cy = 1 - 2*(y^2 + z^2);
yaw = atan2(sy, cy);
sp = 2*(w*y - z*x); sp = max(min(sp,1),-1);
pitch = asin(sp);
sr = 2*(w*x + y*z); cr = 1 - 2*(x^2 + y^2);
roll = atan2(sr, cr);
eul = [yaw pitch roll];
end
