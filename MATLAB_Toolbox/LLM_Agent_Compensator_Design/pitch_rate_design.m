%% Phase 1: Plant Analysis & Linearization
clear; clc; close all;

%% System Parameters
T = 26540;        % Thrust (N)
l_CG = 4.74;      % Nozzle to CG distance (m)
I_y = 21846;      % Pitch moment of inertia (kg*m^2)

% Actuator model
omega_act = 62;   % Actuator natural frequency (rad/s)
zeta_act = 0.505; % Actuator damping ratio

%% Aerodynamic Derivatives (Dimensional)
% Max Q condition
Malpha_maxQ = 15e4;      % [N*m/rad]
Mq_maxQ = -4710;         % [N*m/(rad/s)]

% Launch condition
Malpha_launch = 0;       % [N*m/rad]
Mq_launch = 0;           % [N*m/(rad/s)]

%% Build State-Space Models
% Max Q Plant
a11_maxQ = 0;  % Simplified: neglecting normal force on alpha
a12_maxQ = 1;  % Kinematic coupling: alpha_dot = q
a21_maxQ = Malpha_maxQ / I_y;  % Aero moment effect on q_dot
a22_maxQ = Mq_maxQ / I_y;      % Damping effect on q_dot
b2_maxQ = (T * l_CG) / I_y;    % TVC control authority

A_maxQ = [a11_maxQ, a12_maxQ;
          a21_maxQ, a22_maxQ];
B_maxQ = [0; b2_maxQ];
C_maxQ = [0, 1];  % Output: pitch rate (q)
D_maxQ = 0;

sys_maxQ = ss(A_maxQ, B_maxQ, C_maxQ, D_maxQ);
sys_maxQ.StateName = {'alpha (rad)', 'q (rad/s)'};
sys_maxQ.InputName = 'delta (rad)';
sys_maxQ.OutputName = 'q (rad/s)';

% Launch Plant
A_launch = [0, 1; 0, 0];
B_launch = [0; b2_maxQ];
C_launch = [0, 1];
D_launch = 0;

sys_launch = ss(A_launch, B_launch, C_launch, D_launch);
sys_launch.StateName = {'alpha (rad)', 'q (rad/s)'};
sys_launch.InputName = 'delta (rad)';
sys_launch.OutputName = 'q (rad/s)';

%% Add Actuator Dynamics
s = tf('s');
H_act = omega_act^2 / (s^2 + 2*zeta_act*omega_act*s + omega_act^2);

% Complete plant: Actuator + Rocket
plant_maxQ = H_act * sys_maxQ;
plant_launch = H_act * sys_launch;

%% Convert to Transfer Function for Analysis
[num_maxQ, den_maxQ] = ss2tf(A_maxQ, B_maxQ, C_maxQ, D_maxQ);
G_maxQ = tf(num_maxQ, den_maxQ);

fprintf('=== PLANT TRANSFER FUNCTION (Max Q) ===\n');
fprintf('q/delta = ');
disp(G_maxQ);

%% Analyze Plant Characteristics
fprintf('\n=== PLANT ANALYSIS (Max Q) ===\n');

% Poles and zeros
poles_maxQ = pole(sys_maxQ);
zeros_maxQ = zero(sys_maxQ);

fprintf('\nPoles:\n');
for i = 1:length(poles_maxQ)
    if real(poles_maxQ(i)) > 0
        fprintf('  p%d = %.4f rad/s (UNSTABLE)\n', i, poles_maxQ(i));
    else
        fprintf('  p%d = %.4f rad/s (stable)\n', i, poles_maxQ(i));
    end
end

fprintf('\nZeros:\n');
for i = 1:length(zeros_maxQ)
    fprintf('  z%d = %.4f rad/s\n', i, zeros_maxQ(i));
end

% Unstable pole analysis
p_unstable = max(real(poles_maxQ));
TTD = log(2) / p_unstable;

fprintf('\n--- Instability Analysis ---\n');
fprintf('Unstable pole: %.4f rad/s (%.2f Hz)\n', p_unstable, p_unstable/(2*pi));
fprintf('Time to double: %.4f s (%.1f ms)\n', TTD, TTD*1000);

%% Verify Bode Integral Constraints
omega_target = 20;  % Desired bandwidth (rad/s)

fprintf('\n=== BODE INTEGRAL FEASIBILITY ===\n');
fprintf('Unstable pole: %.2f rad/s\n', p_unstable);
fprintf('Actuator bandwidth: %.0f rad/s\n', omega_act);
fprintf('Desired control bandwidth: %.0f rad/s\n', omega_target);

M_s_predicted = exp(pi * p_unstable / (omega_act - omega_target));
phi_m_predicted = 2 * asin(1/(2*M_s_predicted)) * 180/pi;

fprintf('\nPredicted from Bode integral:\n');
fprintf('  Maximum sensitivity (Ms): %.3f\n', M_s_predicted);
fprintf('  Expected phase margin: %.1f°\n', phi_m_predicted);

if phi_m_predicted >= 50
    fprintf('  ✓ Target specs (PM ≥ 50°) are ACHIEVABLE\n');
else
    fprintf('  ✗ WARNING: Target specs may be difficult\n');
end

%% Plot Plant Characteristics
figure('Name', 'Plant Analysis', 'Position', [100, 100, 1200, 800]);

% Pole-zero map
subplot(2,2,1);
pzmap(sys_maxQ);
grid on;
title('Pole-Zero Map (Max Q)');

% Bode plot
subplot(2,2,2);
bode(sys_maxQ);
grid on;
title('Open-Loop Plant Bode (Max Q, no actuator)');

% Step response (for reference only)
subplot(2,2,3);
step(sys_maxQ, 2);
grid on;
title('Step Response (reference only)');
ylabel('Pitch rate q (rad/s)');

% Nyquist
subplot(2,2,4);
nyquist(sys_maxQ);
grid on;
title('Nyquist Plot (Max Q)');

%% Display Design Recommendations
fprintf('\n=== DESIGN RECOMMENDATIONS ===\n');
fprintf('Rise time requirement: %.1f ms (for 1/5 TTD)\n', TTD/5*1000);
fprintf('Recommended bandwidth: %.0f-%.0f rad/s\n', 3*p_unstable, 5*p_unstable);
fprintf('Target: 20 rad/s = %.1fx unstable pole\n', 20/p_unstable);
fprintf('\nRecommended margins:\n');
fprintf('  Phase margin: ≥ 50° (target 55°)\n');
fprintf('  Gain margin: ≥ 6 dB\n');

%% Phase 2: Controller Synthesis - PI Loop Shaping
clear controller_candidates;
candidates = [];
all_stable = [];

% Suppress control system warnings during search
warning('off', 'Control:analysis:MarginUnstable');
warning('off', 'Control:analysis:NyquistUnstable');

% PI controller structure: C(s) = Kp + Ki/s
fprintf('\n=== CONTROLLER SYNTHESIS: PI Loop Shaping ===\n');
fprintf('Searching for PI gains...\n');

% Expanded search ranges for PI
Kp_range = linspace(0.1, 10, 50);
Ki_range = linspace(0.1, 30, 50);

fprintf('Search space: %d combinations\n', length(Kp_range)*length(Ki_range));

% Progress counter
total_iter = length(Kp_range) * length(Ki_range);
count = 0;

tic;
for i = 1:length(Kp_range)
    for j = 1:length(Ki_range)
        count = count + 1;
        
        Kp = Kp_range(i);
        Ki = Ki_range(j);
        
        try
            % Build PI controller
            C = Kp + Ki/s;
            
            % Open-loop transfer function
            L = C * plant_maxQ;
            
            % Check stability of closed-loop first
            CL = feedback(L, 1);
            if ~isstable(CL)
                continue;
            end
            
            % Check margins
            [Gm, Pm, Wcg, Wcp] = margin(L);
            
            % Handle cases where margin returns array
            if length(Gm) > 1
                Gm = min(Gm);
            end
            if length(Pm) > 1
                Pm = min(Pm);
            end
            
            % Crossover frequency
            wc = Wcp;
            if length(wc) > 1
                wc = max(wc);
            end
            
            % Skip if margins are unrealistic
            if Pm < 0 || Gm < 0
                continue;
            end
            
            % Check sensitivity
            S = 1/(1 + L);
            Ms = norm(S, Inf);
            
            % Compute a "score" for ranking
            pm_penalty = max(0, 50 - Pm);
            gm_penalty = max(0, 6 - 20*log10(Gm));
            bw_penalty = abs(wc - 20);
            ms_penalty = max(0, (Ms - 1.3)*10);
            
            % Total score (lower is better)
            score = pm_penalty + gm_penalty + 0.5*bw_penalty + ms_penalty;
            
            % Store all stable candidates
            all_stable = [all_stable; Kp, Ki, Pm, 20*log10(Gm), wc, Ms, score];
            
            % Also check if fully meets requirements
            if Pm >= 50 && 20*log10(Gm) >= 6 && wc >= 18 && wc <= 25 && Ms < 1.3
                candidates = [candidates; Kp, Ki, Pm, 20*log10(Gm), wc, Ms, score];
            end
            
        catch
            continue;
        end
        
        % Progress update
        if mod(count, 500) == 0
            fprintf('Progress: %.1f%% (%d stable found)\n', 100*count/total_iter, size(all_stable,1));
        end
    end
end
elapsed = toc;

% Re-enable warnings
warning('on', 'Control:analysis:MarginUnstable');
warning('on', 'Control:analysis:NyquistUnstable');

fprintf('Search completed in %.1f seconds\n', elapsed);
fprintf('Total stable controllers found: %d\n', size(all_stable,1));

%% Display Results
if ~isempty(candidates)
    % Sort by score
    [~, idx] = sort(candidates(:,7));
    candidates = candidates(idx, :);
    
    fprintf('\n=== CONTROLLERS MEETING ALL REQUIREMENTS ===\n');
    fprintf('Found %d controllers\n', size(candidates,1));
    fprintf('\nTop 10:\n');
    fprintf('%-8s %-8s %-10s %-10s %-10s %-8s %-8s\n', 'Kp', 'Ki', 'PM (deg)', 'GM (dB)', 'wc (r/s)', 'Ms', 'Score');
    fprintf('---------------------------------------------------------------------------------\n');
    for i = 1:min(10, size(candidates,1))
        fprintf('%-8.3f %-8.3f %-10.1f %-10.1f %-10.1f %-8.3f %-8.2f\n', candidates(i,:));
    end
    
    use_full_spec = true;
else
    fprintf('\n=== NO CONTROLLERS MEETING ALL REQUIREMENTS ===\n');
    fprintf('Showing best stable PI controllers\n\n');
    use_full_spec = false;
end

if ~use_full_spec && ~isempty(all_stable)
    % Sort all stable by score
    [~, idx] = sort(all_stable(:,7));
    all_stable = all_stable(idx, :);
    
    fprintf('\nTop 20 stable PI controllers (closest to requirements):\n');
    fprintf('%-8s %-8s %-10s %-10s %-10s %-8s %-8s\n', 'Kp', 'Ki', 'PM (deg)', 'GM (dB)', 'wc (r/s)', 'Ms', 'Score');
    fprintf('---------------------------------------------------------------------------------\n');
    for i = 1:min(20, size(all_stable,1))
        fprintf('%-8.3f %-8.3f %-10.1f %-10.1f %-10.1f %-8.3f %-8.2f\n', all_stable(i,:));
    end
    
    candidates = all_stable;
end

%% Select and analyze best controller
if ~isempty(candidates)
    Kp_best = candidates(1,1);
    Ki_best = candidates(1,2);
    
    fprintf('\n=== SELECTED PI CONTROLLER ===\n');
    fprintf('Kp = %.4f\n', Kp_best);
    fprintf('Ki = %.4f\n', Ki_best);
    
    % Build final controller
    C_final = Kp_best + Ki_best/s;
    
    % Display transfer function
    fprintf('\nController: C(s) = %.4f + %.4f/s\n', Kp_best, Ki_best);
    
    % Analyze final design
    L_final = C_final * plant_maxQ;
    CL_final = feedback(L_final, 1);
    S_final = 1/(1 + L_final);
    T_final = L_final/(1 + L_final);
    
    [Gm_final, Pm_final, Wcg_final, Wcp_final] = margin(L_final);
    Ms_final = norm(S_final, Inf);
    Mt_final = norm(T_final, Inf);
    
    fprintf('\n=== FINAL PERFORMANCE ===\n');
    fprintf('Phase margin: %.1f° %s\n', Pm_final, ternary(Pm_final >= 50, '✓', '(target: 50°)'));
    fprintf('Gain margin: %.1f dB %s\n', 20*log10(Gm_final), ternary(20*log10(Gm_final) >= 6, '✓', '(target: 6 dB)'));
    fprintf('Crossover frequency: %.1f rad/s (%.2f Hz)\n', Wcp_final, Wcp_final/(2*pi));
    fprintf('Sensitivity peak (Ms): %.3f %s\n', Ms_final, ternary(Ms_final < 1.3, '✓', '(target: <1.3)'));
    fprintf('Complementary sensitivity peak (Mt): %.3f\n', Mt_final);
    
    % Compare to PID
    fprintf('\n=== PI vs PID COMPARISON ===\n');
    fprintf('PI Controller:\n');
    fprintf('  • Simpler structure (2 gains vs 3)\n');
    fprintf('  • Less sensitive to sensor noise\n');
    fprintf('  • Easier to tune in flight\n');
    fprintf('  • Common choice for rockets\n');
    
else
    fprintf('\n✗ No stable controllers found!\n');
end

% Helper function
function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
