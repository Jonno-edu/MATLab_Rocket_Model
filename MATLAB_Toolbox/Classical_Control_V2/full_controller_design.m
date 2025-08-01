clc
clear
close all

%%--- Parameters ---
T = 27607;        % Thrust (N)
l_CG = 5.55;      % Nozzle to CG distance (m)
I_y = 21553;      % Pitch moment of inertia (kg*m^2)
omega_n = 62;     % Actuator natural frequency (rad/s)
zeta = 0.505;     % Actuator damping ratio

%%--- Aerodynamic Pitching Moment Derivative (from flight data) ---
pitching_moment_derivative = 1283950/10; % Nm/rad (from test flight)
aero_gain = pitching_moment_derivative / I_y; % 1/s^2

%%--- Plant: Pitch Rate (including aero) ---
k_plant = T * l_CG / I_y;
plant_with_aero = tf([k_plant], [1 0 -aero_gain]);

%%--- Actuator Model (2nd Order) ---
num_act = [omega_n^2];
den_act = [1 2*zeta*omega_n omega_n^2];
actuator = tf(num_act, den_act);

%%--- Combined Plant: Actuator + Pitch Rate + Aero Instability ---
plant_inner = series(actuator, plant_with_aero);



%--- Calculate Bode and -3dB Cutoff ---
[mag,phase,wout] = bode(plant_inner, logspace(-1,2,1000)); % 0.1 to 100 rad/s
mag_db = squeeze(20*log10(mag));

% Find starting (DC/low freq) magnitude
mag0 = mag_db(1)

% Find frequency where magnitude drops -3dB
idx = find(mag_db <= (mag0-3), 1, 'first');
cutoff_freq = wout(idx)

disp(['Cutoff frequency: ', num2str(cutoff_freq), ' rad/s'])



%%--- Inner PID Design (Pitch Rate Loop) ---
bw_inner = cutoff_freq / 2
opts_inner = pidtuneOptions('PhaseMargin',60,'DesignFocus','balanced');
[C_inner,info_inner] = pidtune(plant_inner,'pid',bw_inner,opts_inner)

%%--- Closed Inner Loop ---
sys_inner_cl = feedback(series(C_inner,plant_inner),1);

