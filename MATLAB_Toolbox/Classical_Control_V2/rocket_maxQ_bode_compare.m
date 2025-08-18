clear; clc; close all;

%% Rocket Pitch Control Design & Analysis

%% --- 1. System Parameters ---
% Burnout conditions (less stable, used for one controller)
T_burnout = 27607;
l_CG_burnout = 5.422;
I_y_burnout = 21300;

% Max-Q conditions (more unstable, used for a second controller)
T_maxQ = 26720;
l_CG_maxQ = 4.75;
I_y_maxQ = 21834;

% Actuator hardware model
omega_act = 62;
zeta_act = 0.505;
s = tf('s');
actuator = tf(omega_act^2, [1, 2*zeta_act*omega_act, omega_act^2]);

%% --- 2. Plant Models at Different Flight Conditions ---
% Aerodynamic parameters
max_q_expected = 79000;
qbar_aero_slope = 1.6831;
max_q_margin = max_q_expected * 1.5;

% Plant at Burnout (no aerodynamic instability)
k_plant_burnout = T_burnout * l_CG_burnout / I_y_burnout;
plant_burnout = tf([k_plant_burnout 0], [1 0 0]);
G_burnout = series(actuator, plant_burnout);

% Plant at Max-Q (aerodynamically unstable)
pitching_moment_derivative_maxQ = max_q_margin * qbar_aero_slope;
% Correctly use the Max-Q moment of inertia here
aero_gain_maxQ = pitching_moment_derivative_maxQ / I_y_maxQ; 
k_plant_maxQ = T_maxQ * l_CG_maxQ / I_y_maxQ;
plant_maxQ = tf([k_plant_maxQ 0], [1 0 -aero_gain_maxQ]);
G_maxQ = series(actuator, plant_maxQ);

%% --- 3. Gain-Scheduled Controller Design ---
% Design parameters
bw_inner_burnout = 0.5; % rad/s
bw_inner_maxQ = 5.5;   % rad/s
opts_inner = pidtuneOptions('PhaseMargin', 70);

% Design controller for Burnout conditions
[C_inner_burnout, ~] = pidtune(G_burnout, 'pi', bw_inner_burnout, opts_inner);

% Design controller for Max-Q conditions
[C_inner_maxQ, ~] = pidtune(G_maxQ, 'pi', bw_inner_maxQ, opts_inner);

fprintf('--- Gain-Scheduled Controllers Designed ---\n');
disp('Burnout Inner Loop Controller (C_inner_burnout):');
C_inner_burnout
disp('Max-Q Inner Loop Controller (C_inner_maxQ):');
C_inner_maxQ

%% --- 4. Analysis of Each Controller on its Respective Plant ---
fprintf('\n--- Analysis at Burnout ---\n');
L_burnout = minreal(series(C_inner_burnout, G_burnout));
[Gm_b, Pm_b, Wcg_b, Wcp_b] = margin(L_burnout);
fprintf('Margins with Burnout Controller on Burnout Plant:\n');
fprintf('  Phase Margin: %.2f deg @ %.2f rad/s\n', Pm_b, Wcp_b);
fprintf('  Gain Margin: %.2f dB @ %.2f rad/s\n', 20*log10(Gm_b), Wcg_b);

fprintf('\n--- Analysis at Max-Q ---\n');
L_maxQ = minreal(series(C_inner_maxQ, G_maxQ));
[Gm_mq, Pm_mq, Wcg_mq, Wcp_mq] = margin(L_maxQ);
fprintf('Margins with Max-Q Controller on Max-Q Plant:\n');
fprintf('  Phase Margin: %.2f deg @ %.2f rad/s\n', Pm_mq, Wcp_mq);
fprintf('  Gain Margin: %.2f dB @ %.2f rad/s\n', 20*log10(Gm_mq), Wcg_mq);

% Check closed-loop stability for each design point
sys_cl_burnout = feedback(L_burnout, 1);
sys_cl_maxQ = feedback(L_maxQ, 1);
fprintf('\n--- Closed-Loop Stability Check ---\n');
fprintf('Is Burnout design stable? %d\n', isstable(sys_cl_burnout));
fprintf('Is Max-Q design stable? %d\n', isstable(sys_cl_maxQ));

%% --- 5. Plots for Analysis ---
figure('Name', 'Burnout Controller on Burnout Plant');
margin(L_burnout);
grid on;
title('Burnout Controller on Burnout Plant');

figure('Name', 'Max-Q Controller on Max-Q Plant');
margin(L_maxQ);
grid on;
title('Max-Q Controller on Max-Q Plant');

