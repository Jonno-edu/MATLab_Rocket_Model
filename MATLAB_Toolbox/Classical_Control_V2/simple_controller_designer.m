clc
clear
close all

%%--- Parameters ---
T = 27607;        % Thrust (N)
l_CG = 5.55;      % Nozzle to CG distance (m)
%I_y = 21553;      % Pitch moment of inertia (kg*m^2)
I_y = 19150;      % Pitch moment of inertia (kg*m^2)
omega_n = 62;     % Actuator natural frequency (rad/s)
zeta = 0.505;     % Actuator damping ratio
V = 1000;          % Forward velocity for lateral plant (m/s)

%%--- Plant: Pitch Rate (no actuator) ---
k_plant = T * l_CG / I_y;
pitch_rate_plant = tf([k_plant], [1 0]);

%%--- Actuator Model (2nd Order) ---
num_act = [omega_n^2];
den_act = [1 2*zeta*omega_n omega_n^2];
actuator = tf(num_act, den_act);

%%--- Combined Plant: Actuator + Pitch Rate ---
plant_inner = series(actuator, pitch_rate_plant);


%--- Calculate Bode and -3dB Cutoff ---
[mag,phase,wout] = bode(plant_inner, logspace(-1,2,1000)); % 0.1 to 100 rad/s
mag_db = squeeze(20*log10(mag));

% Find starting (DC/low freq) magnitude
mag0 = mag_db(1)

% Find frequency where magnitude drops -3dB
idx = find(mag_db <= (mag0-3), 1, 'first');
cutoff_freq = wout(idx)
cutoff_freq = 5

disp(['Cutoff frequency: ', num2str(cutoff_freq), ' rad/s'])


%%--- Inner PID Design (Pitch Rate Loop) ---
%bw_inner = 22.22;
bw_inner = 3 * cutoff_freq
opts_inner = pidtuneOptions('PhaseMargin',60,'DesignFocus','disturbance-rejection');
[C_inner,info_inner] = pidtune(plant_inner,'pid',bw_inner,opts_inner);

%%--- Closed Inner Loop ---
sys_inner_cl = feedback(series(C_inner,plant_inner),1);

%%--- Pitch Angle Plant for Outer Loop ---
plant_outer = series(sys_inner_cl, tf(1,[1 0]));

%%--- Outer PID Design (Pitch Angle Loop) ---
bw_outer = bw_inner / 5
opts_outer = pidtuneOptions('PhaseMargin',60,'DesignFocus','disturbance-rejection');
[C_outer,info_outer] = pidtune(plant_outer,'pi',bw_outer,opts_outer);

%%--- Closed Outer Loop (Pitch Angle) ---
sys_outer_cl = feedback(series(C_outer,plant_outer),1);

%%--- Normal Velocity Plant (Lateral) ---
normal_velocity_plant = sys_outer_cl * V;

%%--- Normal Velocity Controller (Outer, Lateral Velocity) ---
bw_vel = bw_outer/5;
opts_vel = pidtuneOptions('PhaseMargin',60,'DesignFocus','disturbance-rejection');
[C_vel,info_vel] = pidtune(normal_velocity_plant,'pi',bw_vel,opts_vel);
sys_normal_vel_cl = feedback(series(C_vel, normal_velocity_plant), 1);

%%--- Normal Position Plant ---
normal_position_plant = series(sys_normal_vel_cl, tf(1,[1 0]));

%%--- Normal Position Controller (Outer, Lateral Position) ---
bw_pos = bw_vel/4; % Position loop much slower for robustness
opts_pos = pidtuneOptions('PhaseMargin',60,'DesignFocus','disturbance-rejection');
[C_pos,info_pos] = pidtune(normal_position_plant,'pi',bw_pos,opts_pos);
sys_normal_pos_cl = feedback(series(C_pos, normal_position_plant), 1);

%%--- Display Results ---
disp('Inner PID Controller:');
C_inner

disp('Outer PID Controller (Pitch Angle):');
C_outer

disp('Velocity Loop Controller:');
C_vel

disp('Position Loop Controller:');
C_pos

disp('Normal Position Closed Loop (Final Plant):');
sys_normal_pos_cl
