%% STEP 1: Use your PID outer loop as the closed-loop pitch plant
% From your results, best PID controller:
Kp_PID = 0.6552; 
Ki_PID = 0.0253; 
Kd_PID = 0.1643;

C_PID = Kp_PID + Ki_PID/s + Kd_PID*s;

% Closed-loop pitch response (theta_cmd -> theta)
pitch_plant = minreal(perf_maxQ.T * tf(1,[1 0]));
L_pitch = C_PID * pitch_plant;
T_pitch_closed = feedback(L_pitch, 1);

disp('Closed-Loop Pitch Transfer Function:')
disp(T_pitch_closed)

%% STEP 2: Build theta -> aN (lateral acceleration) plant
% Your rocket parameters
m = 980;              
T = 23000;            
V = 100;              
S = 0.2003;           
CN_alpha = 2.0;       
rho = 1;              
q_bar = 0.5*rho*V^2;  
g = 9.81;             

theta_0 = deg2rad(89);  
gamma_0 = deg2rad(86);   
alpha_0 = theta_0 - gamma_0;

% Gain from theta to aN
K_theta = T/m + q_bar*S*CN_alpha/m;
K_gamma = -q_bar*S*CN_alpha/m;

% Plant: theta -> aN
P_theta_to_aN = K_theta * T_pitch_closed;

disp('Plant: theta_cmd -> aN (lateral acceleration)')
disp(P_theta_to_aN)

%% STEP 3: Design Velocity Controller
% Velocity is the integral of acceleration
% State space: [v_N, aN] 
% Input: theta_cmd
% Output: v_N (North velocity)

% aN -> v_N is an integrator
integrator = tf(1, [1 0]);
P_theta_to_vN = P_theta_to_aN * integrator;

disp('Plant: theta_cmd -> v_N (North velocity)')
disp(P_theta_to_vN)

% Design PI velocity controller
bw_velocity = 0.5;  % Much slower than pitch loop
zeta_velocity = 0.7;

% Natural frequency
wn_velocity = bw_velocity / sqrt(1 - 2*zeta_velocity^2 + sqrt(2 - 4*zeta_velocity^2 + 4*zeta_velocity^4));

% PI gains for second-order response
Kp_vel = (2*zeta_velocity*wn_velocity) / dcgain(P_theta_to_vN);
Ki_vel = wn_velocity^2 / dcgain(P_theta_to_vN);

C_velocity = Kp_vel + Ki_vel/s;

% Closed-loop velocity
L_velocity = C_velocity * P_theta_to_vN;
T_velocity = feedback(L_velocity, 1);

disp('Velocity Controller:')
fprintf('Kp_vel = %.4f\n', Kp_vel)
fprintf('Ki_vel = %.4f\n', Ki_vel)

% Analysis
[Gm_vel, Pm_vel, Wcg_vel, Wcp_vel] = margin(L_velocity);
fprintf('Phase Margin: %.2f deg\n', Pm_vel)
fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm_vel))

%% STEP 4: Position Controller (outermost loop)
% v_N -> N_pos is an integrator
P_velocity_to_pos = T_velocity * integrator;

% Design P or PD position controller
Kp_pos = 0.1;   % Position gain
Kd_pos = 0.5;   % Velocity damping

C_position = Kp_pos + Kd_pos*s;

% Closed-loop position
L_position = C_position * P_velocity_to_pos;
T_position = feedback(L_position, 1);

disp('Position Controller:')
fprintf('Kp_pos = %.4f\n', Kp_pos)
fprintf('Kd_pos = %.4f\n', Kd_pos)

% Analysis
[Gm_pos, Pm_pos, Wcg_pos, Wcp_pos] = margin(L_position);
fprintf('Phase Margin: %.2f deg\n', Pm_pos)
fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm_pos))

%% STEP 5: Visualize cascade
figure;

subplot(3,1,1)
step(T_pitch_closed, 2)
grid on
title('Inner Loop: Pitch Response')
ylabel('\theta (rad)')

subplot(3,1,2)
step(T_velocity, 5)
grid on
title('Middle Loop: Velocity Response')
ylabel('v_N (m/s)')

subplot(3,1,3)
step(T_position, 10)
grid on
title('Outer Loop: Position Response')
ylabel('N_{pos} (m)')
xlabel('Time (s)')

%% STEP 6: Bode plots
figure;
subplot(3,1,1)
bode(L_pitch)
grid on
title('Inner Loop: Pitch Loop')

subplot(3,1,2)
bode(L_velocity)
grid on
title('Middle Loop: Velocity Loop')

subplot(3,1,3)
bode(L_position)
grid on
title('Outer Loop: Position Loop')
