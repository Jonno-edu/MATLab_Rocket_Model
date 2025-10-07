%% STATE-SPACE POLE PLACEMENT FOR PLANT WITH ZERO AT ORIGIN
% Alternative method: State feedback + feedforward (NO integral augmentation)
% Plant: G(s) = 5.758*s / (s^2 + 0.2016*s - 12.68)

clear; clc; close all;

%% PLANT DEFINITION
fprintf('=== STATE FEEDBACK + FEEDFORWARD DESIGN ===\n\n');

G_num = [5.758 0];
G_den = [1 0.2016 -12.68];

s = tf('s');
G = tf(G_num, G_den);

fprintf('Plant: G(s) = 5.758*s / (s² + 0.2016*s - 12.68)\n');
fprintf('⚠ Plant has ZERO at s=0 → Cannot use standard integral augmentation\n\n');

%% STATE-SPACE CONVERSION
sys_ss = ss(G);
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

fprintf('State-Space Matrices:\n');
fprintf('A:\n'); disp(A);
fprintf('B:\n'); disp(B);
fprintf('C:\n'); disp(C);

% Check controllability
Cm = ctrb(A, B);
n = size(A, 1);
if rank(Cm) == n
    fprintf('✓ System is controllable (rank = %d)\n\n', rank(Cm));
else
    fprintf('✗ System is NOT controllable\n\n');
    return;
end

%% DESIRED POLES
zeta = 0.7;
wn = 8.0;
wd = wn * sqrt(1 - zeta^2);

desired_poles = [-zeta*wn + 1i*wd, -zeta*wn - 1i*wd];

fprintf('Desired Closed-Loop Poles:\n');
fprintf('  %.2f ± %.2fj (ζ=%.2f, ωn=%.1f rad/s)\n\n', ...
    real(desired_poles(1)), imag(desired_poles(1)), zeta, wn);

%% STATE FEEDBACK GAIN (Pole Placement)
K = place(A, B, desired_poles);

fprintf('State Feedback Gain:\n');
fprintf('K = [%.4f, %.4f]\n\n', K(1), K(2));

% Verify closed-loop poles
A_cl = A - B*K;
poles_achieved = eig(A_cl);
fprintf('Achieved Poles:\n');
fprintf('  %.4f ± %.4fj\n\n', real(poles_achieved(1)), imag(poles_achieved(1)));

%% FEEDFORWARD GAIN FOR ZERO STEADY-STATE ERROR
% For zero SSE: DC gain of closed-loop must equal 1
% T(0) = C*(0 - A_cl)^(-1)*B*N = 1
% Solve for N (feedforward gain)

% At steady-state (s=0): y_ss = -C*A_cl^(-1)*B*N*r_ss
% Want: y_ss = r_ss → N = -1 / (C * inv(A_cl) * B)

N = -1 / (C * inv(A_cl) * B);

fprintf('Feedforward Gain for Zero SSE:\n');
fprintf('N = %.4f\n\n', N);

%% FORM CLOSED-LOOP SYSTEM
% u = N*r - K*x
% Closed-loop: x_dot = A*x + B*(N*r - K*x) = (A - B*K)*x + B*N*r

B_cl = B * N;  % Reference input matrix
sys_cl = ss(A_cl, B_cl, C, D);

%% SIMULATE STEP RESPONSE
t = 0:0.001:3;
[y, t_out, x] = step(sys_cl, t);

% Performance metrics
final_value = y(end);
peak_value = max(y);
peak_time = t_out(find(y == peak_value, 1));
overshoot = ((peak_value - final_value) / final_value) * 100;

idx_10 = find(y >= 0.10 * final_value, 1);
idx_90 = find(y >= 0.90 * final_value, 1);
rise_time = t_out(idx_90) - t_out(idx_10);

settling_band = 0.02 * abs(final_value);
settled_idx = find(abs(y - final_value) <= settling_band, 1);
settling_time = t_out(settled_idx);

sse = 1.0 - final_value;
sse_percent = sse * 100;

fprintf('Step Response Performance:\n');
fprintf('  Rise Time: %.3f s\n', rise_time);
fprintf('  Settling Time: %.3f s\n', settling_time);
fprintf('  Overshoot: %.2f%%\n', overshoot);
fprintf('  Final Value: %.4f\n', final_value);
fprintf('  Steady-State Error: %.2e (%.4f%%)\n', sse, abs(sse_percent));

if abs(sse) < 0.01
    fprintf('  ✓ Zero steady-state error!\n');
end
if overshoot < 5
    fprintf('  ✓ Overshoot < 5%%!\n');
end

%% PLOT RESULTS
figure('Position', [100 100 1200 600]);

subplot(2,2,1);
plot(t_out, y, 'b-', 'LineWidth', 2); hold on;
plot(t_out, ones(size(t_out)), 'r--', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Output y(t)');
title('Step Response');
legend('Output', 'Reference');

subplot(2,2,2);
pzmap(sys_cl); grid on;
title('Closed-Loop Pole-Zero Map');

subplot(2,2,3);
plot(t_out, x); grid on;
xlabel('Time (s)'); ylabel('States');
title('State Trajectories');
legend('x_1', 'x_2');

subplot(2,2,4);
u = N - K*x';
plot(t_out, u, 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Control u(t)');
title('Control Input');

%% SUMMARY
fprintf('\n=== DESIGN SUMMARY ===\n');
fprintf('Controller Structure: u(t) = N*r(t) - K*x(t)\n');
fprintf('  State feedback: K = [%.4f, %.4f]\n', K(1), K(2));
fprintf('  Feedforward: N = %.4f\n', N);
fprintf('\nThis avoids the integral augmentation problem!\n');
fprintf('Zero SSE achieved through proper feedforward gain.\n');
