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

%% Controller Definition - Iteration 2
K = 1.2502;
z1 = 10.00;
p1 = 40.00;
z2 = 2.00;
p2 = 0.20;

num_C = K * conv([1, z1], [1, z2]);
den_C = conv([1, p1], [1, p2]);
C = tf(num_C, den_C);

fprintf('=== CONTROLLER (Iteration 2) ===\n');
fprintf('C(s) = %.4f * (s + %.2f)(s + %.2f) / [(s + %.2f)(s + %.2f)]\n', K, z1, z2, p1, p2);
fprintf('\nLead section: (s + %.2f) / (s + %.2f)\n', z1, p1);
fprintf('Lag section:  (s + %.2f) / (s + %.2f)\n\n', z2, p2);

%% Open-Loop and Closed-Loop Systems
L = C * G;
T = feedback(L, 1);

fprintf('=== STABILITY ANALYSIS ===\n');
poles_CL = pole(T);
fprintf('Closed-loop poles:\n');
for i = 1:length(poles_CL)
    if abs(imag(poles_CL(i))) < 1e-6
        fprintf('  Pole %d: %.4f\n', i, real(poles_CL(i)));
    else
        fprintf('  Pole %d: %.4f ± %.4fj\n', i, real(poles_CL(i)), abs(imag(poles_CL(i))));
    end
end

if all(real(poles_CL) < 0)
    fprintf('\nStatus: STABLE\n\n');
else
    fprintf('\nStatus: UNSTABLE\n');
    unstable_poles = poles_CL(real(poles_CL) >= 0);
    fprintf('Unstable poles: ');
    disp(unstable_poles');
    fprintf('\n');
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
title('Controller C(s) - Lead-Lag');

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

%% Assessment
fprintf('=== ASSESSMENT ===\n');
fprintf('Target crossover: 17.26 rad/s\n');
fprintf('Actual crossover: %.2f rad/s\n', Wcp);
fprintf('Crossover error: %.2f rad/s\n\n', abs(Wcp - 17.26));

if Wcp < 3.46
    fprintf('⚠ Crossover below unstable pole (3.46 rad/s)\n');
    fprintf('  Cannot stabilize - need higher gain\n\n');
elseif Wcp < 13.85
    fprintf('⚠ Crossover below minimum (13.85 rad/s)\n');
    fprintf('  Stability margins may be poor\n\n');
else
    fprintf('✓ Crossover in acceptable range\n\n');
end
