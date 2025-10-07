clear; clc; close all;

%% Plant Definition
num_G = [5.758, 0];
den_G = [1, 0.2016, -12.68];
G = tf(num_G, den_G);

fprintf('=== PLANT ANALYSIS ===\n');
fprintf('G(s) = 5.758s / (s^2 + 0.2016s - 12.68)\n\n');

poles_G = pole(G);
fprintf('Plant poles: %.4f, %.4f\n', poles_G(1), poles_G(2));
fprintf('Plant zeros: 0\n\n');

%% Controller Definition - Iteration 1
Kp = 3.1254;
Ki = 1.7260;

num_C = [Kp, Kp*Ki];
den_C = [1, 0];
C = tf(num_C, den_C);

fprintf('=== CONTROLLER (Iteration 1) ===\n');
fprintf('C(s) = %.4f * (s + %.4f) / s\n\n', Kp, Ki);

%% Open-Loop and Closed-Loop Systems
L = C * G;
T = feedback(L, 1);

fprintf('=== STABILITY ANALYSIS ===\n');
poles_CL = pole(T);
fprintf('Closed-loop poles:\n');
disp(poles_CL);

if all(real(poles_CL) < 0)
    fprintf('Status: STABLE\n\n');
else
    fprintf('Status: UNSTABLE\n\n');
end

%% Stability Margins
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('=== STABILITY MARGINS ===\n');
fprintf('Gain Margin: %.2f dB (at %.2f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Phase Margin: %.2f deg (at %.2f rad/s)\n\n', Pm, Wcp);

%% Bode Plot
figure('Name', 'Bode Plots', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
bode(G);
grid on;
title('Plant G(s)');

subplot(2,2,2);
bode(C);
grid on;
title('Controller C(s)');

subplot(2,2,3:4);
margin(L);
grid on;
title('Open-Loop L(s) = C(s)G(s) with Margins');

%% Step Response
figure('Name', 'Closed-Loop Response', 'Position', [150, 150, 1000, 600]);

if all(real(poles_CL) < 0)
    subplot(1,2,1);
    step(T, 3);
    grid on;
    title('Closed-Loop Step Response');
    xlabel('Time (s)');
    ylabel('Output');
    
    subplot(1,2,2);
    t = 0:0.001:3;
    [y, t] = step(T, t);
    plot(t, y);
    grid on;
    title('Closed-Loop Step Response (Detail)');
    xlabel('Time (s)');
    ylabel('Output');
    
    S = stepinfo(T);
    fprintf('=== STEP RESPONSE CHARACTERISTICS ===\n');
    fprintf('Rise Time: %.4f s\n', S.RiseTime);
    fprintf('Settling Time: %.4f s\n', S.SettlingTime);
    fprintf('Overshoot: %.2f %%\n', S.Overshoot);
    fprintf('Peak: %.4f\n', S.Peak);
    fprintf('Steady-State: %.4f\n\n', y(end));
else
    text(0.5, 0.5, 'System Unstable - No Step Response', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
end

%% Frequency Response at Target Crossover
omega_target = 17.26;
[mag, phase] = bode(G, omega_target);
mag_dB = 20*log10(mag);

fprintf('=== PLANT AT TARGET CROSSOVER (%.2f rad/s) ===\n', omega_target);
fprintf('Magnitude: %.2f dB\n', mag_dB);
fprintf('Phase: %.2f deg\n\n', phase);


