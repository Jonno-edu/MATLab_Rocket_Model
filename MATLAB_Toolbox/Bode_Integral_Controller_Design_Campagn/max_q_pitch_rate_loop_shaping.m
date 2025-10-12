%% Rocket Pitch Rate PI Loop Shaping via Closed-Loop -3dB Bandwidth
clear; clc; close all; 

warning('off','all') % Disable all warnings

%% System Parameters
T = 26540;
l_CG = 4.74;
I_y = 21846;
omega_act = 62;
zeta_act = 0.505;
% Aerodynamic derivatives (Max Q)
Malpha_maxQ = 15e4;
Mq_maxQ = -4710;
% Launch condition (zero velocity, no aero)
T_launch = 23797;
l_CG_launch = 4.018;
I_y_launch = 19150;
Malpha_launch = 0;
Mq_launch = 0;

%% Build Plant Models
% Max Q plant
A_maxQ = [0, 1; Malpha_maxQ/I_y, Mq_maxQ/I_y];
B_maxQ = [0; (T*l_CG)/I_y];
C_maxQ = [0, 1];
D_maxQ = 0;
sys_maxQ = ss(A_maxQ, B_maxQ, C_maxQ, D_maxQ);
% Launch plant
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

%% Plant Analysis
fprintf('=== PLANT ANALYSIS ===\n\n');
% Max Q
poles_maxQ = pole(sys_maxQ);
p_unstable_maxQ = max(real(poles_maxQ));
TTD_maxQ = log(2)/p_unstable_maxQ;
fprintf('Max Q Plant:\n  Unstable pole: %.2f rad/s\n  Time to double: %.1f ms\n', p_unstable_maxQ, TTD_maxQ*1000);

% Launch
poles_launch = pole(sys_launch);
if any(abs(real(poles_launch)) > 1e-6)
    p_unstable_launch = max(real(poles_launch));
    TTD_launch = log(2)/p_unstable_launch;
    fprintf('\nLaunch Plant:\n  Unstable pole: %.2f rad/s\n  Time to double: %.1f ms\n', p_unstable_launch, TTD_launch*1000);
else
    fprintf('\nLaunch Plant:\n  Double integrator (no unstable pole)\n  Pure TVC control, no aerodynamic restoring force\n');
end

%% Bode Integral Limits (Max Q)
omega_target = 30;
M_s_pred = exp(pi * p_unstable_maxQ / (omega_act - omega_target));
fprintf('\nBode integral lower Ms limit (Max Q): %.3f (%.2f dB)\n', M_s_pred, 20*log10(M_s_pred));

%% Controller Grid Search: PI (Design at Max Q)
Kp_range = linspace(0.5, 7, 50);
Ki_range = linspace(0.5, 15, 60);
target_bw = 15;
bw_tol = 2;
results = [];
fprintf('\n=== DESIGNING CONTROLLER AT MAX Q ===\n');
tic;
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
        [Gm,Pm,Wcg,~] = margin(L);
        if isempty(Gm)||isempty(Pm)||Wcg<=0, continue; end
        score = abs(bw_3db-target_bw)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
        results = [results; Kp, Ki, Pm, 20*log10(Gm), bw_3db, Ms, score];
    end 
end
toc;
if isempty(results)
    error('No feasible controllers found. Widen search or relax constraints.');
end
[~,idx] = sort(results(:,7)); results = results(idx,:);
fprintf('\n%-8s %-8s %-10s %-10s %-10s %-8s %-8s\n', ...
    'Kp','Ki','PM (deg)','GM (dB)','BW_3dB','Ms','Score');
disp(results(1:min(10,end),:));

%% Best Controller
Kp_best = results(1,1); Ki_best = results(1,2); C_best = Kp_best + Ki_best/s;
fprintf('\n=== SELECTED CONTROLLER ===\n'); fprintf('Kp = %.4f, Ki = %.4f\n', Kp_best, Ki_best);

%% Performance Analysis at Both Conditions
perf_maxQ.name = 'Max Q';
perf_maxQ.L = C_best * plant_maxQ; perf_maxQ.T = feedback(perf_maxQ.L,1); perf_maxQ.S = feedback(1, perf_maxQ.L);
perf_launch.name = 'Launch';
perf_launch.L = C_best * plant_launch; perf_launch.T = feedback(perf_launch.L,1); perf_launch.S = feedback(1, perf_launch.L);
conditions = {perf_maxQ, perf_launch};
fprintf('\n=== ROBUSTNESS ANALYSIS ===\n');
for k = 1:length(conditions)
    cond = conditions{k};
    fprintf('\n--- %s Condition ---\n', cond.name);
    [Gm,Pm,Wcg,Wcp] = margin(cond.L);
    fprintf('  Gain Margin: %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
    fprintf('  Phase Margin: %.2f deg at %.2f rad/s\n', Pm, Wcp);
    [magS,~,wS] = bode(cond.S,logspace(-1,3,1000));
    magS = squeeze(magS); [Ms_val,idxMs] = max(magS); fMs = wS(idxMs);
    fprintf('  Max Sensitivity (Ms): %.3f at %.2f rad/s\n', Ms_val, fMs);
    [mag_T,~] = bode(cond.T, 0.01); fprintf('  Low-freq gain: %.4f\n', mag_T);
    [mag,~,w] = bode(cond.T,logspace(-1,3,500));
    mag_db = 20*log10(squeeze(mag)); dc_db = mag_db(1);
    idx_bw = find(mag_db <= dc_db - 3, 1);
    if ~isempty(idx_bw)
        fprintf('  Closed-loop BW (-3dB): %.2f rad/s\n', w(idx_bw));
    end
end

%% Outer Loop Pitch Controller Design
fprintf('\n=== OUTER LOOP PITCH CONTROLLER DESIGN ===\n');
integrator = tf(1, [1 0]);
plant_outer = minreal(perf_maxQ.T * integrator);

bw_outer_target = target_bw / 5; bw_outer_tol = 1.5;
Kp_range_outer = linspace(0.1, 5, 30);
Ki_range_outer = linspace(0.01, 1, 20);
Kd_range_outer = linspace(0.01, 1.5, 15);

results_p = []; results_pd = []; results_pi = []; results_pid = [];

% P Controller
for i = 1:length(Kp_range_outer)
    Kp = Kp_range_outer(i);
    C_p = Kp; L = C_p * plant_outer;
    T_CL = feedback(L, 1); S = feedback(1, L);
    [mag,~,w] = bode(T_CL, logspace(-2,2,500));
    mag_db = 20*log10(squeeze(mag)); dc_gain_db = mag_db(1);
    idx_bw = find(mag_db <= dc_gain_db - 3, 1);
    if isempty(idx_bw), continue; end
    bw_3db = w(idx_bw);
    if abs(bw_3db - bw_outer_target) > bw_outer_tol, continue; end
    [Gm,Pm,Wcg,~] = margin(L); Ms = norm(S, Inf);
    score = abs(bw_3db-bw_outer_target)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
    results_p = [results_p; Kp, Pm, 20*log10(Gm), bw_3db, Ms, score];
end

% PD Controller
for i = 1:length(Kp_range_outer)
    for k = 1:length(Kd_range_outer)
        Kp = Kp_range_outer(i); Kd = Kd_range_outer(k);
        C_pd = Kp + Kd*s;
        L = C_pd * plant_outer;
        T_CL = feedback(L, 1); S = feedback(1, L);
        [mag,~,w] = bode(T_CL, logspace(-2,2,500));
        mag_db = 20*log10(squeeze(mag)); dc_gain_db = mag_db(1);
        idx_bw = find(mag_db <= dc_gain_db - 3, 1);
        if isempty(idx_bw), continue; end
        bw_3db = w(idx_bw);
        if abs(bw_3db - bw_outer_target) > bw_outer_tol, continue; end
        [Gm,Pm,Wcg,~] = margin(L); Ms = norm(S, Inf);
        score = abs(bw_3db-bw_outer_target)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
        results_pd = [results_pd; Kp, Kd, Pm, 20*log10(Gm), bw_3db, Ms, score];
    end
end

% PI Controller
for i = 1:length(Kp_range_outer)
    for j = 1:length(Ki_range_outer)
        Kp = Kp_range_outer(i); Ki = Ki_range_outer(j); C_pi = Kp + Ki/s;
        L = C_pi * plant_outer;
        T_CL = feedback(L, 1); S = feedback(1, L);
        [mag,~,w] = bode(T_CL, logspace(-2,2,500));
        mag_db = 20*log10(squeeze(mag)); dc_gain_db = mag_db(1);
        idx_bw = find(mag_db <= dc_gain_db - 3, 1);
        if isempty(idx_bw), continue; end
        bw_3db = w(idx_bw);
        if abs(bw_3db - bw_outer_target) > bw_outer_tol, continue; end
        [Gm,Pm,Wcg,~] = margin(L); Ms = norm(S, Inf);
        score = abs(bw_3db-bw_outer_target)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
        results_pi = [results_pi; Kp, Ki, Pm, 20*log10(Gm), bw_3db, Ms, score];
    end
end

% PID Controller
for i = 1:length(Kp_range_outer)
    for j = 1:length(Ki_range_outer)
        for k = 1:length(Kd_range_outer)
            Kp = Kp_range_outer(i); Ki = Ki_range_outer(j); Kd = Kd_range_outer(k);
            C_pid = Kp + Ki/s + Kd*s;
            L = C_pid * plant_outer;
            T_CL = feedback(L, 1); S = feedback(1, L);
            [mag,~,w] = bode(T_CL, logspace(-2,2,500));
            mag_db = 20*log10(squeeze(mag)); dc_gain_db = mag_db(1);
            idx_bw = find(mag_db <= dc_gain_db - 3, 1);
            if isempty(idx_bw), continue; end
            bw_3db = w(idx_bw);
            if abs(bw_3db - bw_outer_target) > bw_outer_tol, continue; end
            [Gm,Pm,Wcg,~] = margin(L); Ms = norm(S, Inf);
            score = abs(bw_3db-bw_outer_target)+max(0,45-Pm)+max(0,6-20*log10(Gm))+max(0,(Ms-1.2)*10);
            results_pid = [results_pid; Kp, Ki, Kd, Pm, 20*log10(Gm), bw_3db, Ms, score];
        end
    end
end

% List top 10 for each controller
[~, idx_p] = sort(results_p(:,6)); top_p = results_p(idx_p(1:min(10,end)),:);
[~, idx_pd] = sort(results_pd(:,7)); top_pd = results_pd(idx_pd(1:min(10,end)),:);
[~, idx_pi] = sort(results_pi(:,7)); top_pi = results_pi(idx_pi(1:min(10,end)),:);
[~, idx_pid] = sort(results_pid(:,8)); top_pid = results_pid(idx_pid(1:min(10,end)),:);

fprintf('\nTop 10 P controllers:\n'); disp(top_p);
fprintf('\nTop 10 PD controllers:\n'); disp(top_pd);
fprintf('\nTop 10 PI controllers:\n'); disp(top_pi);
fprintf('\nTop 10 PID controllers:\n'); disp(top_pid);

%%
% --- Outer-loop plant for Max Q
plant_outer_maxQ = minreal(perf_maxQ.T * tf(1,[1 0]));
% --- Outer-loop plant for Launch
plant_outer_launch = minreal(perf_launch.T * tf(1,[1 0]));

% Extract top controllers
top_P   = top_p(1,:);
top_PD  = top_pd(1,:);
top_PI  = top_pi(1,:);
top_PID = top_pid(1,:);

fprintf('\n==== OUTER LOOP CONTROLLER TOP RESULTS (Max Q / Launch) ====\n');

% --- P Controller
Kp_P = top_P(1);

for idxPlant = 1:2
    if idxPlant==1, label="Max Q"; plantOL=plant_outer_maxQ;
    else, label="Launch"; plantOL=plant_outer_launch; end

    C_P = Kp_P;
    L_P = C_P * plantOL;
    T_P = feedback(L_P,1);
    S_P = feedback(1, L_P);
    [Gm_P, Pm_P, Wcg_P, Wcp_P] = margin(L_P);
    [y_P, t_P] = step(T_P, 1.5);
    info_P = stepinfo(y_P, t_P);
    Ms_P = norm(S_P, Inf);

    fprintf('\nP Controller - %s:\n  Kp=%.4f\n',label, Kp_P);
    fprintf('  PM: %.2f deg\n  GM: %.2f dB\n', Pm_P, 20*log10(Gm_P));
    fprintf('  BW: %.2f rad/s\n  Ms: %.3f\n', Wcg_P, Ms_P);
    fprintf('  Rise: %.3f s  Overshoot: %.2f%%  Settling: %.3f s\n', ...
        info_P.RiseTime, info_P.Overshoot, info_P.SettlingTime);
end

% --- PD Controller
Kp_PD = top_PD(1); Kd_PD = top_PD(2);

for idxPlant = 1:2
    if idxPlant==1, label="Max Q"; plantOL=plant_outer_maxQ;
    else, label="Launch"; plantOL=plant_outer_launch; end

    C_PD = Kp_PD + Kd_PD*s;
    L_PD = C_PD * plantOL;
    T_PD = feedback(L_PD,1);
    S_PD = feedback(1, L_PD);
    [Gm_PD, Pm_PD, Wcg_PD, Wcp_PD] = margin(L_PD);
    [y_PD, t_PD] = step(T_PD, 1.5);
    info_PD = stepinfo(y_PD, t_PD);
    Ms_PD = norm(S_PD, Inf);

    fprintf('\nPD Controller - %s:\n  Kp=%.4f  Kd=%.4f\n',label, Kp_PD, Kd_PD);
    fprintf('  PM: %.2f deg\n  GM: %.2f dB\n', Pm_PD, 20*log10(Gm_PD));
    fprintf('  BW: %.2f rad/s\n  Ms: %.3f\n', Wcg_PD, Ms_PD);
    fprintf('  Rise: %.3f s  Overshoot: %.2f%%  Settling: %.3f s\n', ...
        info_PD.RiseTime, info_PD.Overshoot, info_PD.SettlingTime);
end

% --- PI Controller
Kp_PI = top_PI(1); Ki_PI = top_PI(2);

for idxPlant = 1:2
    if idxPlant==1, label="Max Q"; plantOL=plant_outer_maxQ;
    else, label="Launch"; plantOL=plant_outer_launch; end

    C_PI = Kp_PI + Ki_PI/s;
    L_PI = C_PI * plantOL;
    T_PI = feedback(L_PI,1);
    S_PI = feedback(1, L_PI);
    [Gm_PI, Pm_PI, Wcg_PI, Wcp_PI] = margin(L_PI);
    [y_PI, t_PI] = step(T_PI, 1.5);
    info_PI = stepinfo(y_PI, t_PI);
    Ms_PI = norm(S_PI, Inf);

    fprintf('\nPI Controller - %s:\n  Kp=%.4f  Ki=%.4f\n',label, Kp_PI, Ki_PI);
    fprintf('  PM: %.2f deg\n  GM: %.2f dB\n', Pm_PI, 20*log10(Gm_PI));
    fprintf('  BW: %.2f rad/s\n  Ms: %.3f\n', Wcg_PI, Ms_PI);
    fprintf('  Rise: %.3f s  Overshoot: %.2f%%  Settling: %.3f s\n', ...
        info_PI.RiseTime, info_PI.Overshoot, info_PI.SettlingTime);
end

% --- PID Controller
Kp_PID = top_PID(1); Ki_PID = top_PID(2); Kd_PID = top_PID(3);

for idxPlant = 1:2
    if idxPlant==1, label="Max Q"; plantOL=plant_outer_maxQ;
    else, label="Launch"; plantOL=plant_outer_launch; end

    C_PID = Kp_PID + Ki_PID/s + Kd_PID*s;
    L_PID = C_PID * plantOL;
    T_PID = feedback(L_PID,1);
    S_PID = feedback(1, L_PID);
    [Gm_PID, Pm_PID, Wcg_PID, Wcp_PID] = margin(L_PID);
    [y_PID, t_PID] = step(T_PID, 1.5);
    info_PID = stepinfo(y_PID, t_PID);
    Ms_PID = norm(S_PID, Inf);

    fprintf('\nPID Controller - %s:\n  Kp=%.4f  Ki=%.4f  Kd=%.4f\n',label, Kp_PID, Ki_PID, Kd_PID);
    fprintf('  PM: %.2f deg\n  GM: %.2f dB\n', Pm_PID, 20*log10(Gm_PID));
    fprintf('  BW: %.2f rad/s\n  Ms: %.3f\n', Wcg_PID, Ms_PID);
    fprintf('  Rise: %.3f s  Overshoot: %.2f%%  Settling: %.3f s\n', ...
        info_PID.RiseTime, info_PID.Overshoot, info_PID.SettlingTime);
end


