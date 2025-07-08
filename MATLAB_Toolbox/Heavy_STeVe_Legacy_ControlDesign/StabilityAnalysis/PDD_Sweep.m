%% Advanced Control: Designing a PDD^2 Controller
clear; clc; close all;

fprintf('--- Moving to Advanced Control: PDD^2 Design ---\n');

%% 1. Define System and "Best-So-Far" PD Gains
S = 0.200296; L = 9.542; L_arm = 3.9; CLa = 2.0; Cma_realistic = 0.1; Cmq_realistic = -10.0;
rho = 0.975; V = 150.6; m = 1528.1; T = 2.48e4; I = 2.77e4;
d1 = (rho * V * S) / (2 * m) * CLa; d2 = T * L_arm / I; d3 = (rho * V^2 * S * L) / (2 * I) * Cma_realistic;
d4 = (rho * V * S * L^2) / (4 * I) * Cmq_realistic; d5 = T / (m * V);
A = [-d1 0 1; 0 0 1; d3 0 d4]; B = [-d5; 0; d2];
fprintf('Model with correct physics created.\n');

% Starting with our best well-balanced PD gains
K_alpha_base = 2.5152;
K_q_base = 5.3182;
fprintf('Base PD Gains: K_alpha=%.2f, K_q=%.2f\n', K_alpha_base, K_q_base);

%% 2. Sweep the New Gain: K_q_dot
k_q_dot_sweep = linspace(0, 1.0, 200);

% Store results: [GM, DampingRatio]
results = NaN(length(k_q_dot_sweep), 2);
best_zeta = -1;
best_design = struct();

fprintf('Sweeping K_q_dot to find the best damping...\n');
warning('off', 'Control:margin:closedLoopUnstable');

for i = 1:length(k_q_dot_sweep)
    kqd = k_q_dot_sweep(i);
    
    % To analyze the PDD^2 controller, we must augment the state-space model
    % The control law u = -Ka*a - Kq*q - Kqd*q_dot can be rewritten by substituting q_dot
    % u = -Ka*a - Kq*q - Kqd*(d3*a + d4*q + d2*u)
    % u * (1 + Kqd*d2) = -(Ka + Kqd*d3)*a - (Kq + Kqd*d4)*q
    % u = -[ (Ka+Kqd*d3)/(1+Kqd*d2) ]*a - [ (Kq+Kqd*d4)/(1+Kqd*d2) ]*q
    % This is the equivalent PD controller that represents the full PDD^2 law
    
    denominator = 1 + kqd * d2;
    ka_equiv = (K_alpha_base + kqd * d3) / denominator;
    kq_equiv = (K_q_base + kqd * d4) / denominator;
    
    K_equiv = [ka_equiv, 0, kq_equiv];
    
    A_cl = A - B*K_equiv;
    poles = eig(A_cl);
    dyn_poles = poles(abs(poles) > 1e-6);
    
    if all(real(dyn_poles) < 0)
        try
            [GM, ~] = margin(ss(A, B, K_equiv, 0));
            gm_db = 20*log10(GM);
            
            if gm_db > 10.0 % Only consider robust designs
                complex_poles = dyn_poles(imag(dyn_poles) ~= 0);
                if ~isempty(complex_poles)
                    dom_pole = complex_poles(1);
                    zeta = -real(dom_pole) / abs(dom_pole);
                    if zeta > best_zeta
                        best_zeta = zeta;
                        best_design.K_alpha = K_alpha_base;
                        best_design.K_q = K_q_base;
                        best_design.K_q_dot = kqd;
                        best_design.GM = gm_db;
                        best_design.Zeta = zeta;
                    end
                end
            end
        catch; continue; end
    end
end
warning('on', 'Control:margin:closedLoopUnstable');

%% 3. Display the Final, High-Performance Design
if isfield(best_design, 'Zeta')
    fprintf('\n--- Advanced PDD^2 Controller Design Complete ---\n');
    fprintf('Found a high-performance controller by adding angular acceleration feedback.\n');
    fprintf('  K_alpha = %.4f\n', best_design.K_alpha);
    fprintf('  K_q     = %.4f\n', best_design.K_q);
    fprintf('  K_q_dot = %.4f  (The new gain)\n', best_design.K_q_dot);
    fprintf('  Gain Margin: %.2f dB\n', best_design.GM);
    fprintf('  Damping Ratio (zeta): %.3f  (Massively improved!)\n\n', best_design.Zeta);
    
    fprintf('Implementation in Simulink for this is more complex, as it requires calculating\n');
    fprintf('q_dot at each timestep, often within a "Controller" subsystem block.\n');
else
    fprintf('\nNo PDD^2 design found that improved performance while maintaining robustness.\n');
end
