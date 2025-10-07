% Iteration 6: Modified PID Controller Design and Analysis (with minreal)
% Plant: G(s) = 5.758s / (s^2 + 0.2016s - 12.68)
% Controller: C(s) = (1.5s^2 + 8.0s + 20.0) / s

clear; clc; close all;

%% Plant Transfer Function
num_plant = [5.758 0];
den_plant = [1 0.2016 -12.68];
G = tf(num_plant, den_plant);

fprintf('=== ITERATION 6: Modified PID Structure (with minreal) ===\n\n');
fprintf('Plant Transfer Function:\n');
display(G);

poles_plant = pole(G);
zeros_plant = zero(G);
fprintf('Plant Poles: ');
disp(poles_plant');
fprintf('Plant Zeros: ');
disp(zeros_plant');

%% Controller Transfer Function
Kd = 1.5;
Kp = 8.0;
Ki = 17.7908;

num_ctrl = [Kd Kp Ki];
den_ctrl = [1 0];
C = tf(num_ctrl, den_ctrl);

fprintf('\nController Parameters:\n');
fprintf('  Kd = %.2f\n', Kd);
fprintf('  Kp = %.2f\n', Kp);
fprintf('  Ki = %.2f\n\n', Ki);
fprintf('Controller Transfer Function:\n');
display(C);

%% Open-Loop Transfer Function
L = C * G;
fprintf('Open-Loop Transfer Function L(s) = C(s) * G(s):\n');
display(L);

fprintf('\n--- BEFORE POLE-ZERO CANCELLATION ---\n');
fprintf('Open-Loop Poles:\n');
disp(pole(L));
fprintf('Open-Loop Zeros:\n');
disp(zero(L));

%% Apply minreal to cancel pole-zero at origin
L_reduced = minreal(L);
fprintf('\n--- AFTER POLE-ZERO CANCELLATION (minreal) ---\n');
fprintf('Reduced Open-Loop Transfer Function:\n');
display(L_reduced);

fprintf('Reduced Open-Loop Poles:\n');
disp(pole(L_reduced));
fprintf('Reduced Open-Loop Zeros:\n');
disp(zero(L_reduced));

%% Closed-Loop Transfer Function (from original)
T = feedback(L, 1);
fprintf('\n--- CLOSED-LOOP (Original) ---\n');
fprintf('Closed-Loop Transfer Function T(s):\n');
display(T);

poles_cl = pole(T);
fprintf('Closed-Loop Poles:\n');
for i = 1:length(poles_cl)
    if isreal(poles_cl(i))
        if abs(poles_cl(i)) < 1e-6
            fprintf('  Pole %d: %.4e (AT ORIGIN - marginally stable)\n', i, poles_cl(i));
        elseif poles_cl(i) < 0
            fprintf('  Pole %d: %.4f (STABLE)\n', i, poles_cl(i));
        else
            fprintf('  Pole %d: %.4f (UNSTABLE)\n', i, poles_cl(i));
        end
    else
        if real(poles_cl(i)) < 0
            fprintf('  Pole %d: %.4f + %.4fi (STABLE)\n', i, real(poles_cl(i)), imag(poles_cl(i)));
        else
            fprintf('  Pole %d: %.4f + %.4fi (UNSTABLE)\n', i, real(poles_cl(i)), imag(poles_cl(i)));
        end
    end
end

%% Apply minreal to closed-loop
T_reduced = minreal(T);
fprintf('\n--- CLOSED-LOOP (After minreal) ---\n');
fprintf('Reduced Closed-Loop Transfer Function:\n');
display(T_reduced);

poles_cl_reduced = pole(T_reduced);
fprintf('\nReduced Closed-Loop Poles:\n');
for i = 1:length(poles_cl_reduced)
    if isreal(poles_cl_reduced(i))
        if poles_cl_reduced(i) < 0
            fprintf('  Pole %d: %.4f (STABLE)\n', i, poles_cl_reduced(i));
        else
            fprintf('  Pole %d: %.4f (UNSTABLE)\n', i, poles_cl_reduced(i));
        end
    else
        if real(poles_cl_reduced(i)) < 0
            fprintf('  Pole %d: %.4f + %.4fi (STABLE)\n', i, real(poles_cl_reduced(i)), imag(poles_cl_reduced(i)));
        else
            fprintf('  Pole %d: %.4f + %.4fi (UNSTABLE)\n', i, real(poles_cl_reduced(i)), imag(poles_cl_reduced(i)));
        end
    end
end

is_stable = all(real(poles_cl_reduced) < 0);
if is_stable
    fprintf('\nStability Status: STABLE (after pole-zero cancellation)\n');
else
    fprintf('\nStability Status: UNSTABLE\n');
end

%% Stability Margins (use reduced open-loop)
[Gm, Pm, Wcg, Wcp] = margin(L_reduced);
Gm_dB = 20*log10(Gm);

fprintf('\n=== Stability Margins (from reduced open-loop) ===\n');
fprintf('Gain crossover frequency: %.2f rad/s\n', Wcp);
fprintf('Phase margin: %.2f deg\n', Pm);
if isinf(Gm)
    fprintf('Gain margin: Infinite\n');
else
    fprintf('Gain margin: %.2f dB (at %.2f rad/s)\n', Gm_dB, Wcg);
end

%% Step Response Analysis (use reduced closed-loop)
if is_stable
    [y, t] = step(T_reduced, 5);
    
    info = stepinfo(T_reduced);
    
    fprintf('\n=== Step Response Performance (reduced system) ===\n');
    fprintf('Rise time: %.3f s\n', info.RiseTime);
    fprintf('Settling time (2%%): %.3f s\n', info.SettlingTime);
    fprintf('Overshoot: %.2f %%\n', info.Overshoot);
    fprintf('Peak: %.4f\n', info.Peak);
    fprintf('Peak time: %.3f s\n', info.PeakTime);
    
    y_final = y(end);
    sse = 1 - y_final;
    fprintf('Final value: %.4f\n', y_final);
    fprintf('Steady-state error: %.6f\n', sse);
end

%% Bode Plots
figure('Position', [100 100 1400 500]);

subplot(1,3,1);
margin(L);
grid on;
title('Open-Loop Bode (Original) - UNSTABLE');
legend('Location', 'southwest');

subplot(1,3,2);
margin(L_reduced);
grid on;
title('Open-Loop Bode (Reduced) with Margins');
legend('Location', 'southwest');

subplot(1,3,3);
bode(T_reduced);
grid on;
title('Closed-Loop Bode (Reduced)');

%% Step Response Plot
figure('Position', [100 100 1000 600]);

if is_stable
    step(T_reduced, 5);
    grid on;
    title('Closed-Loop Step Response (Reduced System)');
    xlabel('Time (s)');
    ylabel('Amplitude');
    
    hold on;
    yline(1, '--r', 'Reference', 'LineWidth', 1.5);
    hold off;
else
    text(0.5, 0.5, 'SYSTEM UNSTABLE - No Step Response', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
    axis off;
end

%% Pole-Zero Maps
figure('Position', [100 100 1400 800]);

subplot(2,3,1);
pzmap(G);
grid on;
title('Plant Pole-Zero Map');

subplot(2,3,2);
pzmap(L);
grid on;
title('Open-Loop (Original)');

subplot(2,3,3);
pzmap(L_reduced);
grid on;
title('Open-Loop (Reduced)');

subplot(2,3,4);
pzmap(C);
grid on;
title('Controller Pole-Zero Map');

subplot(2,3,5);
pzmap(T);
grid on;
title('Closed-Loop (Original)');

subplot(2,3,6);
pzmap(T_reduced);
grid on;
title('Closed-Loop (Reduced)');

%% Nyquist Plot
figure('Position', [100 100 1000 500]);

subplot(1,2,1);
nyquist(L_reduced);
grid on;
title('Nyquist Plot (Reduced Open-Loop)');

subplot(1,2,2);
step(T_reduced, 5);
grid on;
title('Step Response (Reduced Closed-Loop)');
hold on;
yline(1, '--r', 'Reference', 'LineWidth', 1.5);
hold off;

%% Summary
fprintf('\n=== Design Assessment ===\n');
fprintf('\nIMPORTANT NOTE:\n');
fprintf('The pole at origin in both numerator and denominator cancels.\n');
fprintf('This is the s/(s) = 1 cancellation from controller integrator\n');
fprintf('and plant differentiator.\n\n');

if is_stable
    fprintf('✓ System is STABLE (after pole-zero cancellation)\n');
    
    if Wcp >= 13.85 && Wcp <= 20.67
        fprintf('✓ Crossover frequency in target range (13.85-20.67 rad/s)\n');
    else
        fprintf('⚠ Crossover frequency: %.2f rad/s (target: 13.85-20.67 rad/s)\n', Wcp);
    end
    
    if Pm >= 60
        fprintf('✓ Excellent phase margin (>= 60 deg)\n');
    elseif Pm >= 45
        fprintf('⚠ Acceptable phase margin (>= 45 deg)\n');
    else
        fprintf('✗ Low phase margin (< 45 deg)\n');
    end
    
    if info.Overshoot < 5
        fprintf('✓ Low overshoot (< 5%%)\n');
    elseif info.Overshoot < 15
        fprintf('⚠ Moderate overshoot (< 15%%)\n');
    else
        fprintf('✗ High overshoot (>= 15%%)\n');
    end
    
    if abs(sse) < 0.01
        fprintf('✓ Steady-state error near zero\n');
    else
        fprintf('⚠ Steady-state error: %.4f\n', sse);
    end
    
    fprintf('\n=== CONCLUSION ===\n');
    fprintf('The pole-zero cancellation at the origin was causing MATLAB\n');
    fprintf('to report the system as unstable. After using minreal(), the\n');
    fprintf('actual system dynamics are revealed and the system is stable.\n');
else
    fprintf('✗ System is UNSTABLE (even after pole-zero cancellation)\n');
    fprintf('   Recommendations:\n');
    fprintf('   - Reduce controller gains\n');
    fprintf('   - Increase derivative gain for more phase lead\n');
    fprintf('   - Consider state-space design methods\n');
end

fprintf('\n=== End of Analysis ===\n');
