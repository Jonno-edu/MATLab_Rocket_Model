%% Rocket Pitch Control Design & Analysis

%% --- 1. System Parameters & Constraints ---
% Vehicle physical properties
T = 27607;        % Thrust (N)
l_CG = 5.422;     % Nozzle to CG distance (m)
I_y = 21300;      % Pitch moment of inertia (kg*m^2)
% Actuator hardware model
omega_act = 62;   % Actuator natural frequency (rad/s)
zeta_act = 0.505; % Actuator damping ratio

%% --- 2. Plant Definition ---
% Aero derivatives at operating points
max_q_expected = 78000;
max_q_margin = max_q_expected * 2;
qbar_aero_slope = 1.6831;

pitching_moment_derivative_maxQ = max_q_margin * qbar_aero_slope; % At max Q
pitching_moment_derivative_launch = 0;    % At launch

aero_gain_maxQ = pitching_moment_derivative_maxQ / I_y;
aero_gain_launch = pitching_moment_derivative_launch / I_y;

k_plant = T*l_CG/I_y;

num_plant = [k_plant 0];
den_launch = [1 0 -aero_gain_launch];
den_maxQ = [1 0 -aero_gain_maxQ];

plant_launch = tf(num_plant, den_launch);
plant_maxQ   = tf(num_plant, den_maxQ);

% Actuator transfer function
num_act = [omega_act^2];
den_act = [1 2*zeta_act*omega_act omega_act^2];
actuator = tf(num_act, den_act);

plant_inner_open_loop_launch = series(actuator, plant_launch);
plant_inner_open_loop_maxQ   = series(actuator, plant_maxQ);

%% --- 3. Bandwidths and Sampling Times ---
bw_inner_launch = 0.5;
bw_inner_maxQ   = 9;
bw_outer_launch = bw_inner_launch / 5;
bw_outer_maxQ   = bw_inner_maxQ   / 5;

% Sample rates, 10x bandwidth, rad/s to Hz conversion
% Ts_inner = 1 / (10 * (bw_inner_maxQ/(2*pi))); % inner loop (always use fastest)
% Ts_outer = 1 / (10 * (bw_outer_maxQ/(2*pi))); % outer loop (always use fastest)
Ts_inner = 1/200;
Ts_outer = Ts_inner*5;

wn_ramp = 0.2;


%% --- 4. Inner Loop Controllers ---
opts_inner = pidtuneOptions('PhaseMargin', 60);
[C_inner_launch, ~] = pidtune(plant_inner_open_loop_launch, 'pi', bw_inner_launch, opts_inner);
[C_inner_maxQ,   ~] = pidtune(plant_inner_open_loop_maxQ,   'pi', bw_inner_maxQ,   opts_inner);

sys_inner_cl_launch = feedback(series(C_inner_launch, plant_inner_open_loop_launch), 1);
sys_inner_cl_maxQ   = feedback(series(C_inner_maxQ,   plant_inner_open_loop_maxQ),   1);

%% --- 5. Outer Loop Controllers ---
plant_outer_open_loop_launch = series(sys_inner_cl_launch, tf(1,[1 0]));
plant_outer_open_loop_maxQ   = series(sys_inner_cl_maxQ,   tf(1,[1 0]));

opts_outer = pidtuneOptions('PhaseMargin', 60);
[C_outer_launch, ~] = pidtune(plant_outer_open_loop_launch, 'pidf', bw_outer_launch, opts_outer);
[C_outer_maxQ,   ~] = pidtune(plant_outer_open_loop_maxQ,   'pi', bw_outer_maxQ,   opts_outer);

sys_outer_cl_launch = feedback(series(C_outer_launch, plant_outer_open_loop_launch), 1);
sys_outer_cl_maxQ   = feedback(series(C_outer_maxQ,   plant_outer_open_loop_maxQ),   1);

%% --- 6. Discretize Controllers ---
C_inner_launch_d = c2d(C_inner_launch, Ts_inner, 'tustin');
C_inner_maxQ_d   = c2d(C_inner_maxQ,   Ts_inner, 'tustin');
C_outer_launch_d = c2d(C_outer_launch, Ts_outer, 'tustin');
C_outer_maxQ_d   = c2d(C_outer_maxQ,   Ts_outer, 'tustin');

%% --- 7. Export Discrete Controllers to Workspace ---
assignin('base', 'C_inner_launch_d', C_inner_launch_d);
assignin('base', 'C_inner_maxQ_d',   C_inner_maxQ_d);
assignin('base', 'C_outer_launch_d', C_outer_launch_d);
assignin('base', 'C_outer_maxQ_d',   C_outer_maxQ_d);



%% --- 7.1 Export Discrete PID Gains to Workspace ---
% Inner loop (Launch)
C_inner_launch_d_Kp = C_inner_launch_d.Kp;
C_inner_launch_d_Ki = C_inner_launch_d.Ki;
C_inner_launch_d_Kd = C_inner_launch_d.Kd;
assignin('base', 'C_inner_launch_d_Kp', C_inner_launch_d_Kp);
assignin('base', 'C_inner_launch_d_Ki', C_inner_launch_d_Ki);
assignin('base', 'C_inner_launch_d_Kd', C_inner_launch_d_Kd);

% Inner loop (Max Q)
C_inner_maxQ_d_Kp = C_inner_maxQ_d.Kp;
C_inner_maxQ_d_Ki = C_inner_maxQ_d.Ki;
C_inner_maxQ_d_Kd = C_inner_maxQ_d.Kd;
C_inner_maxQ_d_Tf = C_inner_maxQ_d.Tf;
assignin('base', 'C_inner_maxQ_d_Kp', C_inner_maxQ_d_Kp);
assignin('base', 'C_inner_maxQ_d_Ki', C_inner_maxQ_d_Ki);
assignin('base', 'C_inner_maxQ_d_Kd', C_inner_maxQ_d_Kd);
assignin('base', 'C_inner_maxQ_d_Tf', C_inner_maxQ_d_Tf);

% Outer loop (Launch)
C_outer_launch_d_Kp = C_outer_launch_d.Kp;
C_outer_launch_d_Ki = C_outer_launch_d.Ki;
C_outer_launch_d_Kd = C_outer_launch_d.Kd;
C_outer_launch_d_N = 1 / C_outer_launch_d.Tf;
assignin('base', 'C_outer_launch_d_Kp', C_outer_launch_d_Kp);
assignin('base', 'C_outer_launch_d_Ki', C_outer_launch_d_Ki);
assignin('base', 'C_outer_launch_d_Kd', C_outer_launch_d_Kd);
assignin('base', 'C_outer_launch_d_N', C_outer_launch_d_N);

% Outer loop (Max Q)
C_outer_maxQ_d_Kp = C_outer_maxQ_d.Kp;
C_outer_maxQ_d_Ki = C_outer_maxQ_d.Ki;
C_outer_maxQ_d_Kd = C_outer_maxQ_d.Kd;
assignin('base', 'C_outer_maxQ_d_Kp', C_outer_maxQ_d_Kp);
assignin('base', 'C_outer_maxQ_d_Ki', C_outer_maxQ_d_Ki);
assignin('base', 'C_outer_maxQ_d_Kd', C_outer_maxQ_d_Kd);

%% Sensor Modelling

% Multi-rate gyro filter design
fs_gyro = 800;        % Gyro sensor rate
fs_filter = 400;       % Filter processing rate  
fs_control = 200;      % Control loop rate

% Design filter at filter processing rate
fc_gyro = 20;          % Cutoff frequency (Hz)
wc = fc_gyro * 2 * pi; % Convert to rad/s
[b, a] = butter(2, wc, 's');
[b_gyro_fast, a_gyro_fast] = bilinear(b, a, fs_filter);

% Export coefficients
assignin('base', 'b_gyro_fast', b_gyro_fast);
assignin('base', 'a_gyro_fast', a_gyro_fast);
assignin('base', 'Ts_gyro_filter', 1/fs_filter);

