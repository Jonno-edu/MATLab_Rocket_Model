%% ControlAgent-Perplexity System Analysis
% Analysis of the successfully designed controller
% System: G(s) = 14.53/s^2 (double integrator)
% Controller: Loop shaping design with omega_L=6.75, beta_b=2.69, beta_l=5.0

clear; clc; close all;

%% System Definition
% Plant: G(s) = 14.53/s^2
G_num = 14.53;
G_den = [1 0 0];  % s^2
G = tf(G_num, G_den);

%% Controller Parameters (from successful design)
omega_L = 6.75;
beta_b = 2.69;
beta_l = 5.0;

% Calculate K_p (proportional gain)
% K_p = 1/|G(j*omega_L)|
s_jw = 1j * omega_L;
G_jw = evalfr(G, s_jw);
K_p = 1 / abs(G_jw);

fprintf('=== Controller Design Parameters ===\n');
fprintf('omega_L = %.2f rad/s\n', omega_L);
fprintf('beta_b = %.3f\n', beta_b);
fprintf('beta_l = %.3f\n', beta_l);
fprintf('K_p = %.3f\n', K_p);
fprintf('\n');

%% Controller Components
% K_i(s) = (beta_b*s + omega_L) / (s * sqrt(beta_b^2 + 1))
Ki_num = [beta_b, omega_L];
Ki_den = [sqrt(beta_b^2 + 1), 0];  % s in denominator
K_i = tf(Ki_num, Ki_den);

% K_l(s) = (beta_l*s + omega_L) / (s + beta_l*omega_L)
Kl_num = [beta_l, omega_L];
Kl_den = [1, beta_l * omega_L];
K_l = tf(Kl_num, Kl_den);

% Complete controller: K(s) = K_p * K_i(s) * K_l(s)
K = K_p * K_i * K_l;

fprintf('=== Transfer Functions ===\n');
fprintf('Plant G(s):\n');
G

fprintf('Integral Controller K_i(s):\n');
K_i

fprintf('Lead Controller K_l(s):\n');
K_l

fprintf('Complete Controller K(s):\n');
K

%% Loop Transfer Function
L = G * K;  % L(s) = G(s) * K(s)

fprintf('Loop Transfer Function L(s) = G(s)*K(s):\n');
L

%% Closed-Loop Transfer Function
% T(s) = L(s) / (1 + L(s))
T = feedback(L, 1);

fprintf('Closed-Loop Transfer Function T(s):\n');
T

%% Pole-Zero Analysis
figure(1);
subplot(2,2,1);
pzmap(G);
title('Plant G(s) = 14.53/s^2 - Pole-Zero Map');
grid on;

subplot(2,2,2);
pzmap(K);
title('Controller K(s) - Pole-Zero Map');
grid on;

subplot(2,2,3);
pzmap(L);
title('Open-Loop L(s) = G(s)K(s) - Pole-Zero Map');
grid on;

subplot(2,2,4);
pzmap(T);
title('Closed-Loop T(s) - Pole-Zero Map');
grid on;

%% Frequency Response Analysis
figure(2);

% Bode plot of loop transfer function
subplot(2,1,1);
margin(L);
title('Open-Loop Bode Plot L(s) = G(s)K(s)');
grid on;

% Get margins
[Gm, Pm, Wgm, Wpm] = margin(L);
Gm_dB = 20*log10(Gm);

fprintf('=== Stability Margins ===\n');
fprintf('Phase Margin: %.2f degrees (at %.2f rad/s)\n', Pm, Wpm);
fprintf('Gain Margin: %.2f dB (at %.2f rad/s)\n', Gm_dB, Wgm);
fprintf('\n');

% Closed-loop frequency response
subplot(2,1,2);
bode(T);
title('Closed-Loop Bode Plot T(s)');
grid on;

%% Time Domain Analysis
figure(3);

% Step response
subplot(2,2,1);
[y_step, t_step] = step(T);
step(T);
title('Closed-Loop Step Response');
grid on;

% Calculate performance metrics
stepinfo_results = stepinfo(T);
fprintf('=== Step Response Performance ===\n');
fprintf('Rise Time: %.3f s\n', stepinfo_results.RiseTime);
fprintf('Settling Time: %.3f s\n', stepinfo_results.SettlingTime);
fprintf('Overshoot: %.2f%%\n', stepinfo_results.Overshoot);
fprintf('Peak: %.3f\n', stepinfo_results.Peak);
fprintf('Peak Time: %.3f s\n', stepinfo_results.PeakTime);
fprintf('\n');

% Impulse response
subplot(2,2,2);
impulse(T);
title('Closed-Loop Impulse Response');
grid on;

% Ramp response (multiply by 1/s)
ramp_input = tf(1, [1 0]);  % 1/s
T_ramp = T * ramp_input;
subplot(2,2,3);
[y_ramp, t_ramp] = step(T_ramp);
plot(t_ramp, y_ramp, 'b-', t_ramp, t_ramp, 'r--');
title('Closed-Loop Ramp Response');
xlabel('Time (s)'); ylabel('Output');
legend('System Response', 'Ideal Ramp', 'Location', 'best');
grid on;

% Control effort (controller output)
subplot(2,2,4);
T_control = feedback(K, G);  % Controller output / Reference input
step(T_control);
title('Controller Output (Control Effort)');
grid on;

%% Root Locus Analysis
figure(4);
rlocus(G*K_i*K_l);  % Root locus varying K_p
title('Root Locus Analysis (varying K_p)');
grid on;

%% Nyquist Plot
figure(5);
nyquist(L);
title('Nyquist Plot of L(s) = G(s)K(s)');
grid on;

%% Performance Summary
fprintf('=== DESIGN VERIFICATION ===\n');
fprintf('Target Phase Margin: >= 45 degrees\n');
fprintf('Achieved Phase Margin: %.2f degrees ✓\n', Pm);
fprintf('\n');
fprintf('Target Settling Time: <= 2.0 seconds\n');
fprintf('Achieved Settling Time: %.3f seconds ✓\n', stepinfo_results.SettlingTime);
fprintf('\n');
fprintf('Steady-State Error (step): %.6f ✓\n', 1 - dcgain(T));
fprintf('\n');

if Pm >= 45 && stepinfo_results.SettlingTime <= 2.0
    fprintf('🎉 ALL REQUIREMENTS MET! Excellent design!\n');
else
    fprintf('⚠️  Some requirements not met.\n');
end

%% Additional Analysis: Sensitivity Functions
figure(6);

% Sensitivity function S(s) = 1/(1+L(s))
S = feedback(1, L);

% Complementary sensitivity T(s) = L(s)/(1+L(s)) (already calculated)

% Control sensitivity function
CS = feedback(K, G);

subplot(2,2,1);
bodemag(S);
title('Sensitivity Function |S(jω)|');
grid on;

subplot(2,2,2);
bodemag(T);
title('Complementary Sensitivity |T(jω)|');
grid on;

subplot(2,2,3);
bodemag(CS);
title('Control Sensitivity |CS(jω)|');
grid on;

subplot(2,2,4);
bodemag(S, T, CS);
legend('S(s)', 'T(s)', 'CS(s)', 'Location', 'best');
title('All Sensitivity Functions');
grid on;

%% Save results
save('control_system_analysis.mat', 'G', 'K', 'L', 'T', 'omega_L', 'beta_b', 'beta_l', 'K_p', 'Pm', 'stepinfo_results');

fprintf('Analysis complete! Results saved to control_system_analysis.mat\n');
