%% FINAL, DEFINITIVE ANALYSIS: 2D Stability Mapping
clear; clc; close all;

fprintf('--- Final Analysis: 2D Stability Mapping of K_alpha and K_q ---\n');

%% 1. Define System Parameters with Realistic, Controllable Aerodynamics
S = 0.200296; L = 9.542; L_arm = 3.9; CLa = 2.0;
rho = 0.975; V = 150.6; m = 1528.1; T = 2.48e4; I = 2.77e4;

Cma_realistic = 0.1;    % Mildly unstable static margin
Cmq_realistic = -10.0;  % Realistic aerodynamic damping
fprintf('Using Realistic Airframe: Cma = %.2f, Cmq = %.1f\n', Cma_realistic, Cmq_realistic);

%% 2. Build the Model with All Physics Corrections
d1 = (rho * V * S) / (2 * m) * CLa;
d2 = T * L_arm / I;
d3 = (rho * V^2 * S * L) / (2 * I) * Cma_realistic;
d4 = (rho * V * S * L^2) / (4 * I) * Cmq_realistic; % Corrected denominator
d5 = T / (m * V);

A_full = [-d1 0 1; 0 0 1; d3 0 d4];
B_full = [-d5; 0; d2];
fprintf('Model with all physics corrections created.\n');

%% 3. 2D Gain Sweep Setup
k_alpha_sweep = linspace(0.1, 5, 100);
k_q_sweep = linspace(0.1, 10, 120);

gm_map = NaN(length(k_q_sweep), length(k_alpha_sweep));

fprintf('Performing 2D sweep across %d K_alpha and %d K_q values...\n', length(k_alpha_sweep), length(k_q_sweep));

% Suppress the expected instability warning for a clean output
warning('off', 'Control:margin:closedLoopUnstable');

%% 4. The 2D Sweep Loop
for i = 1:length(k_alpha_sweep)
    for j = 1:length(k_q_sweep)
        ka = k_alpha_sweep(i);
        kq = k_q_sweep(j);
        
        K_full = [ka, 0, kq];
        
        A_cl = A_full - B_full * K_full;
        poles = eig(A_cl);
        
        % Correct stability check: dynamic poles must be stable
        dynamic_poles = poles(abs(poles) > 1e-6);
        if all(real(dynamic_poles) < 0)
            try
                [GM, ~] = margin(ss(A_full, B_full, K_full, 0));
                gm_map(j, i) = 20*log10(GM);
            catch
                continue;
            end
        end
    end
end
fprintf('Sweep complete. Plotting stability map...\n');

% Re-enable the warning if desired for subsequent code
warning('on', 'Control:margin:closedLoopUnstable');

%% 5. Plot the Stability Map and Report Results
figure;
contourf(k_alpha_sweep, k_q_sweep, gm_map, 20, 'LineColor', 'none');
colorbar;
hold on;
contour(k_alpha_sweep, k_q_sweep, gm_map, [6 6], 'r', 'LineWidth', 2);

title('Robustness Map: Gain Margin (dB) vs. Controller Gains');
xlabel('Proportional Gain, K_{alpha}');
ylabel('Rate Damping Gain, K_q');
legend('Gain Margin (dB)', 'Robust Stability Boundary (6 dB)');
grid on;
set(gca, 'Layer', 'top');

[max_gm, idx] = max(gm_map(:));
if ~isnan(max_gm) && max_gm > 6
    [j_best, i_best] = ind2sub(size(gm_map), idx);
    k_alpha_best = k_alpha_sweep(i_best);
    k_q_best = k_q_sweep(j_best);
    
    fprintf('\n\n--- BREAKTHROUGH: ROBUST DESIGN FOUND ---\n');
    fprintf('A region of robust stability has been identified.\n\n');
    fprintf('Optimal Controller Gains:\n');
    fprintf('  K_alpha = %.4f\n', k_alpha_best);
    fprintf('  K_q     = %.4f\n', k_q_best);
    fprintf('Resulting Gain Margin: %.2f dB\n', max_gm);
else
    fprintf('\n\nNo robust design region found. The airframe parameters are still too challenging.\n');
end

