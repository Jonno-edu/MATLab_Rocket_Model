%% Rocket Pitch Rate PI Loop Shaping via Closed-Loop -3dB Bandwidth
clear; clc; close all; 
warning('off','all')

%% System Parameters
T = 26540;
l_CG = 4.74;
I_y = 21846;
omega_act = 62;
zeta_act = 0.505;

% Aerodynamic derivatives (Max Q)
% Malpha_maxQ = 15e4;
% Mq_maxQ = -4710;

Malpha_maxQ = 22e4;
Mq_maxQ = -5000;

% Launch condition
T_launch = 23797;
l_CG_launch = 4.018;
I_y_launch = 19150;
Malpha_launch = 0;
Mq_launch = 0;

%% Build Plant Models
A_maxQ = [0, 1; Malpha_maxQ/I_y, Mq_maxQ/I_y];
B_maxQ = [0; (T*l_CG)/I_y];
C_maxQ = [0, 1];
D_maxQ = 0;
sys_maxQ = ss(A_maxQ, B_maxQ, C_maxQ, D_maxQ);

A_launch = [0, 1; Malpha_launch/I_y_launch, Mq_launch/I_y_launch];
B_launch = [0; (T_launch*l_CG_launch)/I_y_launch];
C_launch = [0, 1];
D_launch = 0;
sys_launch = ss(A_launch, B_launch, C_launch, D_launch);

% Add actuator dynamics
s = tf('s');
H_act = omega_act^2 / (s^2 + 2*zeta_act*omega_act*s + omega_act^2);
plant_maxQ = H_act * sys_maxQ;
plant_launch = H_act * sys_launch;

%% Sample Times
Ts_inner = 1/200;
Ts_outer = Ts_inner/5;

fprintf('=== CONTROLLER DESIGN ===\n');
fprintf('Inner loop (rate): %.0f Hz (Ts = %.3f s)\n', 1/Ts_inner, Ts_inner);
fprintf('Outer loop (attitude): %.0f Hz (Ts = %.3f s)\n\n', 1/Ts_outer, Ts_outer);

%% Inner Loop PI Controller Design (at Max Q)
target_bw = 15;
bw_tol = 2;

Kp_range = linspace(0.5, 7, 50);
Ki_range = linspace(0.5, 15, 60);
results = [];

fprintf('Designing inner loop PI controller at Max Q...\n');
for i = 1:length(Kp_range)
    for j = 1:length(Ki_range)
        Kp = Kp_range(i);
        Ki = Ki_range(j);
        C = Kp + Ki/s;
        L = C * plant_maxQ;
        T_CL = feedback(L,1);
        
        [mag,~,w] = bode(T_CL,logspace(-1,3,500));
        mag = squeeze(mag); mag_db = 20*log10(mag);
        dc_gain_db = mag_db(1);
        target_db = dc_gain_db - 3;
        idx = find(mag_db <= target_db, 1);
        if isempty(idx), continue; end
        
        bw_3db = w(idx);
        if abs(bw_3db - target_bw) > bw_tol, continue; end
        
        S = feedback(1, L);
        Ms = norm(S, Inf);
        [Gm,Pm,~,~] = margin(L);
        if isempty(Gm)||isempty(Pm), continue; end
        
        score = abs(bw_3db-target_bw)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
        results = [results; Kp, Ki, Pm, 20*log10(Gm), bw_3db, Ms, score];
    end 
end

if isempty(results)
    error('No feasible inner loop controllers found.');
end

[~,idx] = sort(results(:,7));
Kp_inner = results(idx(1),1);
Ki_inner = results(idx(1),2);
C_inner = Kp_inner + Ki_inner/s;

fprintf('\n=== INNER LOOP PI CONTROLLER (Continuous) ===\n');
fprintf('Kp = %.4f\n', Kp_inner);
fprintf('Ki = %.4f\n', Ki_inner);
fprintf('PM = %.2f deg, GM = %.2f dB\n', results(idx(1),3), results(idx(1),4));
fprintf('BW = %.2f rad/s, Ms = %.3f\n', results(idx(1),5), results(idx(1),6));

%% Discretize Inner Loop using Tustin
C_inner_discrete = c2d(C_inner, Ts_inner, 'tustin');
[num_inner, den_inner] = tfdata(C_inner_discrete, 'v');

fprintf('\n=== INNER LOOP DISCRETE PI (Tustin, Ts=%.3fs) ===\n', Ts_inner);
fprintf('Numerator: [%.6f, %.6f]\n', num_inner(1), num_inner(2));
fprintf('Denominator: [%.6f, %.6f]\n', den_inner(1), den_inner(2));

% Extract discrete PID coefficients for implementation
% Discrete PI: u[k] = u[k-1] + Kp*(e[k]-e[k-1]) + Ki*Ts*e[k]
Kp_inner_d = num_inner(1) - num_inner(2);
Ki_inner_d = (num_inner(1) + num_inner(2)) / Ts_inner / 2;

fprintf('Implementation form:\n');
fprintf('  Kp_discrete = %.6f\n', Kp_inner_d);
fprintf('  Ki_discrete = %.6f\n', Ki_inner_d);

%% Build Outer Loop Plant (Max Q and Launch)
L_inner_maxQ = C_inner * plant_maxQ;
T_inner_maxQ = feedback(L_inner_maxQ, 1);
plant_outer_maxQ = minreal(T_inner_maxQ * tf(1,[1 0]));

L_inner_launch = C_inner * plant_launch;
T_inner_launch = feedback(L_inner_launch, 1);
plant_outer_launch = minreal(T_inner_launch * tf(1,[1 0]));

%% Outer Loop PD Controller Design
bw_outer_target = target_bw / 5;
bw_outer_tol = 1.5;

Kp_range_outer = linspace(0.1, 5, 40);
Kd_range_outer = linspace(0.01, 1.5, 30);
results_pd = [];

fprintf('\n\nDesigning outer loop PD controller...\n');
for i = 1:length(Kp_range_outer)
    for k = 1:length(Kd_range_outer)
        Kp = Kp_range_outer(i);
        Kd = Kd_range_outer(k);
        C_pd = Kp + Kd*s;
        L = C_pd * plant_outer_maxQ;
        T_CL = feedback(L, 1);
        S = feedback(1, L);
        
        [mag,~,w] = bode(T_CL, logspace(-2,2,500));
        mag_db = 20*log10(squeeze(mag));
        dc_gain_db = mag_db(1);
        idx_bw = find(mag_db <= dc_gain_db - 3, 1);
        if isempty(idx_bw), continue; end
        
        bw_3db = w(idx_bw);
        if abs(bw_3db - bw_outer_target) > bw_outer_tol, continue; end
        
        [Gm,Pm,~,~] = margin(L);
        Ms = norm(S, Inf);
        
        score = abs(bw_3db-bw_outer_target)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
        results_pd = [results_pd; Kp, Kd, Pm, 20*log10(Gm), bw_3db, Ms, score];
    end
end

if isempty(results_pd)
    error('No feasible outer loop PD controllers found.');
end

[~,idx] = sort(results_pd(:,7));
Kp_outer = results_pd(idx(1),1);
Kd_outer = results_pd(idx(1),2);
C_outer = Kp_outer + Kd_outer*s;

fprintf('\n=== OUTER LOOP PD CONTROLLER (Continuous) ===\n');
fprintf('Kp = %.4f\n', Kp_outer);
fprintf('Kd = %.4f\n', Kd_outer);
fprintf('PM = %.2f deg, GM = %.2f dB\n', results_pd(idx(1),3), results_pd(idx(1),4));
fprintf('BW = %.2f rad/s, Ms = %.3f\n', results_pd(idx(1),5), results_pd(idx(1),6));

%% Discretize Outer Loop using Tustin
C_outer_discrete = c2d(C_outer, Ts_outer, 'tustin');
[num_outer, den_outer] = tfdata(C_outer_discrete, 'v');

fprintf('\n=== OUTER LOOP DISCRETE PD (Tustin, Ts=%.3fs) ===\n', Ts_outer);
fprintf('Numerator: [%.6f, %.6f]\n', num_outer(1), num_outer(2));
fprintf('Denominator: [%.6f, %.6f]\n', den_outer(1), den_outer(2));

% Extract discrete PD coefficients
Kp_outer_d = (num_outer(1) + num_outer(2)) / 2;
Kd_outer_d = (num_outer(1) - num_outer(2)) * Ts_outer / 2;

fprintf('Implementation form:\n');
fprintf('  Kp_discrete = %.6f\n', Kp_outer_d);
fprintf('  Kd_discrete = %.6f\n', Kd_outer_d);

%% Performance Analysis at Both Conditions
fprintf('\n\n=== PERFORMANCE ANALYSIS ===\n');

for cond_idx = 1:2
    if cond_idx == 1
        label = 'Max Q';
        L_outer = C_outer * plant_outer_maxQ;
    else
        label = 'Launch';
        L_outer = C_outer * plant_outer_launch;
    end
    
    T_outer = feedback(L_outer, 1);
    S_outer = feedback(1, L_outer);
    
    fprintf('\n--- %s Condition (Outer Loop) ---\n', label);
    [Gm,Pm,Wcg,Wcp] = margin(L_outer);
    fprintf('  Gain Margin: %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
    fprintf('  Phase Margin: %.2f deg at %.2f rad/s\n', Pm, Wcp);
    
    Ms_val = norm(S_outer, Inf);
    fprintf('  Max Sensitivity (Ms): %.3f\n', Ms_val);
    
    [y, t] = step(T_outer, 2);
    info = stepinfo(y, t);
    fprintf('  Rise Time: %.3f s\n', info.RiseTime);
    fprintf('  Overshoot: %.2f%%\n', info.Overshoot);
    fprintf('  Settling Time: %.3f s\n', info.SettlingTime);
end

%% Summary for C Implementation
fprintf('\n\n');
fprintf('==================================================\n');
fprintf('      FINAL CONTROLLER GAINS FOR C CODE          \n');
fprintf('==================================================\n');
fprintf('\n--- INNER LOOP (Rate Control, 100 Hz) ---\n');
fprintf('Continuous:\n');
fprintf('  kp_inner_pitch = %.6f;\n', Kp_inner);
fprintf('  ki_inner_pitch = %.6f;\n', Ki_inner);
fprintf('  kp_inner_yaw   = %.6f;\n', Kp_inner);
fprintf('  ki_inner_yaw   = %.6f;\n', Ki_inner);
fprintf('  kd_inner_pitch = 0.0;  // PI only\n');
fprintf('  kd_inner_yaw   = 0.0;  // PI only\n');

fprintf('\n--- OUTER LOOP (Attitude Control, 20 Hz) ---\n');
fprintf('Continuous:\n');
fprintf('  kp_outer_pitch = %.6f;\n', Kp_outer);
fprintf('  kd_outer_pitch = %.6f;\n', Kd_outer);
fprintf('  kp_outer_yaw   = %.6f;\n', Kp_outer);
fprintf('  kd_outer_yaw   = %.6f;\n', Kd_outer);
fprintf('\n==================================================\n');
