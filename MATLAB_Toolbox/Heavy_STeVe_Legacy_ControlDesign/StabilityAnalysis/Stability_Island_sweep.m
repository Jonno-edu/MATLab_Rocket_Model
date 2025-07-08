%% Performance Tuning Part 2: High-Resolution Narrow Search
clear; clc; close all;

fprintf('--- Phase 1: Broad Search to Identify Promising Region ---\n');

%% 1. Define System Parameters and Build the Model (t=35s)
S = 0.200296; L = 9.542; L_arm = 3.9; CLa = 2.0; Cma_realistic = 0.1; Cmq_realistic = -10.0;
rho = 0.975; V = 150.6; m = 1528.1; T = 2.48e4; I = 2.77e4;
d1 = (rho * V * S) / (2 * m) * CLa; d2 = T * L_arm / I; d3 = (rho * V^2 * S * L) / (2 * I) * Cma_realistic;
d4 = (rho * V * S * L^2) / (4 * I) * Cmq_realistic; d5 = T / (m * V);
A_full = [-d1 0 1; 0 0 1; d3 0 d4]; B_full = [-d5; 0; d2];
fprintf('Model with correct physics created.\n');

%% 2. Broad Stability Map
k_alpha_broad = linspace(0.1, 5, 80);
k_q_broad = linspace(0.1, 10, 100);
gm_map_broad = NaN(length(k_q_broad), length(k_alpha_broad));
warning('off', 'Control:margin:closedLoopUnstable');
for i = 1:length(k_alpha_broad); for j = 1:length(k_q_broad)
    K = [k_alpha_broad(i), 0, k_q_broad(j)]; A_cl = A_full - B_full*K;
    poles = eig(A_cl); dyn_poles = poles(abs(poles)>1e-6);
    if all(real(dyn_poles)<0); try; [GM,~]=margin(ss(A_full,B_full,K,0)); gm_map_broad(j,i)=20*log10(GM); catch; continue; end; end
end; end
fprintf('Broad map complete.\n\n');

%% 3. Find Centroid of Broad Region
[robust_j, robust_i] = find(gm_map_broad > 10);
k_alpha_center = k_alpha_broad(round(mean(robust_i)));
k_q_center = k_q_broad(round(mean(robust_j)));
fprintf('--- Phase 2: High-Resolution Search around the Promising Region ---\n');
fprintf('Centering narrow search around K_alpha=%.2f, K_q=%.2f\n', k_alpha_center, k_q_center);

%% 4. Define and Run Narrow Search
search_radius_percent = 0.3; % Search +/- 30% around the center point
k_alpha_narrow = linspace(k_alpha_center*(1-search_radius_percent), k_alpha_center*(1+search_radius_percent), 80);
k_q_narrow = linspace(k_q_center*(1-search_radius_percent), k_q_center*(1+search_radius_percent), 100);

% Store results: [GM, DampingRatio]
perf_map = NaN(length(k_q_narrow), length(k_alpha_narrow), 2);

for i = 1:length(k_alpha_narrow)
    for j = 1:length(k_q_narrow)
        ka = k_alpha_narrow(i); kq = k_q_narrow(j);
        K = [ka, 0, kq]; A_cl = A_full - B_full*K;
        poles = eig(A_cl); dyn_poles = poles(abs(poles)>1e-6);
        if all(real(dyn_poles)<0)
            try
                [GM,~]=margin(ss(A_full,B_full,K,0));
                perf_map(j,i,1) = 20*log10(GM);
                % Find dominant complex poles to calculate damping ratio
                complex_poles = dyn_poles(imag(dyn_poles) ~= 0);
                if ~isempty(complex_poles)
                    dom_pole = complex_poles(1);
                    zeta = -real(dom_pole) / abs(dom_pole);
                    perf_map(j,i,2) = zeta;
                end
            catch; continue; end
        end
    end
end
fprintf('Narrow search complete.\n\n');

%% 5. Find the Optimal Performance Design
robustness_floor_dB = 10.0;
% Create a mask of all designs that are sufficiently robust
valid_mask = perf_map(:,:,1) > robustness_floor_dB;
% Get the damping ratios of only the valid designs
valid_zetas = perf_map(:,:,2);
valid_zetas(~valid_mask) = -1; % Invalidate non-robust points

% Find the location of the best (highest) damping ratio
[best_zeta, idx_best_perf] = max(valid_zetas(:));
if ~isnan(best_zeta) && best_zeta > 0
    [j_bp, i_bp] = ind2sub(size(valid_zetas), idx_best_perf);
    k_alpha_bp = k_alpha_narrow(i_bp);
    k_q_bp = k_q_narrow(j_bp);
    gm_bp = perf_map(j_bp,i_bp,1);

    fprintf('--- Final "Best Performance" Design ---\n');
    fprintf('Found robust gains with the best damping ratio.\n');
    fprintf('  K_alpha = %.4f\n', k_alpha_bp);
    fprintf('  K_q     = %.4f\n', k_q_bp);
    fprintf('  Gain Margin: %.2f dB\n', gm_bp);
    fprintf('  Damping Ratio (zeta): %.3f\n\n', best_zeta);
    
    % Prepare workspace for the final Simulink run
    fprintf('Updating workspace with the FINAL controller gains.\n');
    K = [k_alpha_bp, 0, k_q_bp];
else
    fprintf('No robust designs found in the narrow search.\n');
    K = [k_alpha_center, 0, k_q_center]; % Fallback
end

%% 6. Plot the New Map and Prepare for Simulation
figure;
contourf(k_alpha_narrow, k_q_narrow, perf_map(:,:,2), 20, 'LineColor', 'none');
h = colorbar; ylabel(h, 'Damping Ratio (\zeta)');
hold on;
contour(k_alpha_narrow, k_q_narrow, perf_map(:,:,1), [robustness_floor_dB robustness_floor_dB], 'r', 'LineWidth', 2);
plot(k_alpha_bp, k_q_bp, 'wx', 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', 'Best Performance');
title('Performance Map: Damping Ratio vs. Controller Gains');
xlabel('Proportional Gain, K_{alpha}'); ylabel('Rate Damping Gain, K_q');
legend('Damping Ratio', 'Robustness Boundary (10 dB)', 'Best Performance', 'Location', 'northeast');
grid on; set(gca, 'Layer', 'top');

initial_pitch_rate_rad_s = 0.1;
x0 = [0; 0; initial_pitch_rate_rad_s];
fprintf('\n--- Workspace Ready ---\n');
fprintf('Re-run your Simulink model now to see the final, high-performance response.\n');
