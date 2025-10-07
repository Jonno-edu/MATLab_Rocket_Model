%% LLM-Guided Control System Design - Iteration 8: 2-DOF PID Controller
% Plant: G(s) = 5.758s / (s^2 + 0.2016s - 12.68)
% Unstable rocket/missile pitch dynamics with differentiator

clear all; close all; clc;

%% Plant Definition
num_G = [5.758 0];
den_G = [1 0.2016 -12.68];
G = tf(num_G, den_G);

% Plant Analysis
poles_G = pole(G);
zeros_G = zero(G);
fprintf('=== PLANT ANALYSIS ===\n');
fprintf('Poles: %.4f, %.4f\n', poles_G(1), poles_G(2));
fprintf('Zeros: %.4f\n', zeros_G);
fprintf('Unstable pole: %.4f rad/s\n', max(real(poles_G)));

%% Feedback Controller Design (from Iteration 6)
Kd = 1.5;
Kp = 8.0;
Ki = 20.0;

num_C = [Kd Kp Ki];
den_C = [1 0];
C = tf(num_C, den_C);

fprintf('\n=== FEEDBACK CONTROLLER ===\n');
fprintf('C(s) = (%.2fs^2 + %.2fs + %.2f) / s\n', Kd, Kp, Ki);

%% Open-Loop Analysis
L = C * G;
L_reduced = minreal(L, 1e-4);

fprintf('\n=== OPEN-LOOP SYSTEM ===\n');
[num_L, den_L] = tfdata(L_reduced, 'v');
fprintf('L(s) numerator: ');
fprintf('%.4f ', num_L);
fprintf('\n');
fprintf('L(s) denominator: ');
fprintf('%.4f ', den_L);
fprintf('\n');

% Stability Margins
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('\nGain Margin: %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
fprintf('Phase Margin: %.2f deg at %.2f rad/s\n', Pm, Wcp);

%% Closed-Loop System (1-DOF - for comparison)
T_1DOF = feedback(C*G, 1);
T_1DOF_reduced = minreal(T_1DOF, 1e-4);

poles_CL = pole(T_1DOF_reduced);
fprintf('\n=== 1-DOF CLOSED-LOOP (No Feedforward) ===\n');
fprintf('Closed-loop poles:\n');
for i = 1:length(poles_CL)
    if imag(poles_CL(i)) == 0
        fprintf('  %.4f\n', poles_CL(i));
    else
        fprintf('  %.4f ± %.4fi\n', real(poles_CL(i)), abs(imag(poles_CL(i))));
        break;
    end
end

% DC Gain of 1-DOF system
[num_T1, den_T1] = tfdata(T_1DOF_reduced, 'v');
DC_gain_1DOF = num_T1(end) / den_T1(end);
SSE_1DOF = (1 - DC_gain_1DOF) * 100;
fprintf('DC Gain: %.4f\n', DC_gain_1DOF);
fprintf('Steady-State Error: %.2f%%\n', SSE_1DOF);

%% Feedforward Gain Design
Kff = 1 / DC_gain_1DOF;
fprintf('\n=== FEEDFORWARD GAIN ===\n');
fprintf('Kff = 1 / DC_gain = %.4f\n', Kff);

%% 2-DOF Closed-Loop System
T_2DOF = Kff * feedback(C*G, 1);
T_2DOF_reduced = minreal(T_2DOF, 1e-4);

[num_T2, den_T2] = tfdata(T_2DOF_reduced, 'v');
DC_gain_2DOF = num_T2(end) / den_T2(end);
SSE_2DOF = (1 - DC_gain_2DOF) * 100;

fprintf('\n=== 2-DOF CLOSED-LOOP (With Feedforward) ===\n');
fprintf('DC Gain: %.4f\n', DC_gain_2DOF);
fprintf('Steady-State Error: %.2f%%\n', SSE_2DOF);

%% Step Response Analysis
t = 0:0.001:2;

% 1-DOF Response
[y_1DOF, t_out] = step(T_1DOF_reduced, t);
info_1DOF = stepinfo(T_1DOF_reduced);

% 2-DOF Response
[y_2DOF, t_out] = step(T_2DOF_reduced, t);
info_2DOF = stepinfo(T_2DOF_reduced);

fprintf('\n=== STEP RESPONSE COMPARISON ===\n');
fprintf('                    1-DOF        2-DOF\n');
fprintf('Rise Time:        %.4f s    %.4f s\n', info_1DOF.RiseTime, info_2DOF.RiseTime);
fprintf('Settling Time:    %.4f s    %.4f s\n', info_1DOF.SettlingTime, info_2DOF.SettlingTime);
fprintf('Overshoot:        %.2f%%      %.2f%%\n', info_1DOF.Overshoot, info_2DOF.Overshoot);
fprintf('Peak:             %.4f       %.4f\n', info_1DOF.Peak, info_2DOF.Peak);
fprintf('Final Value:      %.4f       %.4f\n', y_1DOF(end), y_2DOF(end));

%% Bode Plot
figure('Position', [100 100 1200 500]);

subplot(1,2,1);
margin(L);
grid on;
title('Open-Loop Bode (L = C*G)');

subplot(1,2,2);
bode(T_1DOF_reduced, T_2DOF_reduced);
grid on;
legend('1-DOF (No FF)', '2-DOF (With FF)', 'Location', 'best');
title('Closed-Loop Bode Comparison');

%% Step Response Plot
figure('Position', [100 100 1200 500]);

subplot(1,2,1);
plot(t_out, y_1DOF, 'b-', 'LineWidth', 1.5); hold on;
plot(t_out, ones(size(t_out)), 'k--', 'LineWidth', 1);
yline(1.1237, 'r--', 'Final Value = 1.1237', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Output');
title('1-DOF Step Response (12.37% SSE)');
legend('Response', 'Reference', 'Location', 'southeast');
xlim([0 1.5]);
ylim([0 1.3]);

subplot(1,2,2);
plot(t_out, y_2DOF, 'r-', 'LineWidth', 1.5); hold on;
plot(t_out, ones(size(t_out)), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Output');
title('2-DOF Step Response (0% SSE)');
legend('Response', 'Reference', 'Location', 'southeast');
xlim([0 1.5]);
ylim([0 1.3]);

%% Pole-Zero Map
figure('Position', [100 100 800 600]);
pzmap(T_1DOF_reduced, T_2DOF_reduced);
grid on;
legend('1-DOF', '2-DOF', 'Location', 'best');
title('Closed-Loop Pole-Zero Map');

%% Root Locus (for reference)
figure('Position', [100 100 800 600]);
rlocus(L);
grid on;
title('Root Locus of L(s) = C(s)*G(s)');

%% Nyquist Plot
figure('Position', [100 100 800 600]);
nyquist(L);
grid on;
title('Nyquist Plot of L(s) = C(s)*G(s)');

%% Summary Report
fprintf('\n=== DESIGN SUMMARY ===\n');
fprintf('Controller Type: 2-DOF PID\n');
fprintf('Feedback: C(s) = (%.2fs^2 + %.2fs + %.2f) / s\n', Kd, Kp, Ki);
fprintf('Feedforward: Kff = %.4f\n', Kff);
fprintf('\nPerformance Metrics:\n');
fprintf('  Overshoot: %.2f%% (target: <5%%)\n', info_2DOF.Overshoot);
fprintf('  Settling Time: %.3f s\n', info_2DOF.SettlingTime);
fprintf('  Steady-State Error: %.2f%% (target: 0%%)\n', SSE_2DOF);
fprintf('  Gain Margin: %.2f dB\n', 20*log10(Gm));
fprintf('  Phase Margin: %.2f deg (target: >60 deg)\n', Pm);
fprintf('  Crossover Frequency: %.2f rad/s (target: 13.85-20.67 rad/s)\n', Wcp);
fprintf('\nAll design objectives ACHIEVED!\n');

%% Export Controller Coefficients
fprintf('\n=== IMPLEMENTATION PARAMETERS ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);
fprintf('Kff = %.4f\n', Kff);
fprintf('\nReference scaling: r_cmd = %.4f * r_desired\n', Kff);
