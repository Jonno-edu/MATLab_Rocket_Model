clc; clear; close all;
s = tf('s');
t = 0:0.01:10;

%% --- System Parameters ---
T = 27607;
l_CG = 5.422;
I_y = 21300;
pitching_moment_derivative = 103000;
omega_act = 62;
zeta_act = 0.505;

aero_gain = pitching_moment_derivative / I_y;
k_plant = T * l_CG / I_y;

plant_with_aero = tf([k_plant 0], [1 0 -aero_gain]);
actuator = tf([omega_act^2], [1 2*zeta_act*omega_act omega_act^2]);
P_pitch = series(actuator, plant_with_aero);

%% --- PID Controller
bw_inner = 15; 
opts_inner = pidtuneOptions('PhaseMargin', 60); 
[C_inner, ~] = pidtune(P_pitch, 'pid', bw_inner, opts_inner);
sys_inner_cl = feedback(C_inner * P_pitch, 1);
sys_inner_ol = minreal(P_pitch*C_inner);


stepinfo(0.1*sys_inner_cl)




bw_outer = bw_inner/5; 
opts_outer = pidtuneOptions('PhaseMargin', 30); 
[C_outer, ~] = pidtune(sys_inner_cl, 'pi', bw_inner, opts_inner);
sys_outer_cl = feedback(C_inner * P_pitch, 1);
sys_outer_ol = minreal(sys_inner_cl*C_outer);
