%% 1. Data Extraction
logsout_data = simOut.logsout;

% Extract top-level signals by their exact names from your logs
theta_var_ts = logsout_data.getElement('theta_variation_deg').Values;
t_sim   = theta_var_ts.Time;
theta   = theta_var_ts.Data;

V       = logsout_data.getElement('airspeed').Values.Data;
I       = logsout_data.getElement('I').Values.Data;              % [N x 3] (or check your shape)
Cma     = logsout_data.getElement('Cmalpha').Values.Data;        % Per rad
L_arm   = logsout_data.getElement('CG_X').Values.Data;           % [m]
S       = logsout_data.getElement('Lref').Values.Data;           % Use your real reference area if you have (else Lref)
% Air density is inside the ENV bus
env_struct = logsout_data.getElement('ENV').Values;
rho     = env_struct.AirDensity.Data;

%% 2. Analytical Time to Double — Evaluate at Max-Q
q_dyn = 0.5 * rho .* V.^2;
[~, idx_eval] = max(q_dyn);                % At maximum dynamic pressure

% Inertia vector shape: check if [3 x N] or [N x 3]
if size(I, 2) == 3
    I_pitch = I(2, 2);              % If [N x 3]
else
    I_pitch = I(2, idx_eval);              % If [3 x N]
end

V_val    = V(idx_eval);
rho_val  = rho(idx_eval);
L_val    = L_arm(idx_eval);
S_val    = S(idx_eval);
Cma_val  = Cma(idx_eval);

% Aero pitching moment derivative (Nm/rad)
M_alpha = 0.5 * rho_val * V_val^2 * S_val * L_val * Cma_val;

T_double = log(2) * sqrt(I_pitch / abs(M_alpha));
fprintf('Classical time-to-double: %.4f s (evaluated at t = %.2f s)\n', T_double, t_sim(idx_eval));

%% 3. Find Doubling Points in Sim
targets = [0.5, 1, 2, 4];
t_marks = zeros(1, numel(targets));
idx_marks = zeros(1, numel(targets));
for k = 1:numel(targets)
    idx = find(theta >= targets(k), 1, 'first');
    if isempty(idx)
        error('Theta never reaches %.2f deg', targets(k));
    end
    t_marks(k) = t_sim(idx);
    idx_marks(k) = idx;
end

%% 4. Window for Plotting
idx_start = find(theta >= 0.01, 1, 'first');
idx_end   = find(theta >= 6, 1, 'first');
if isempty(idx_start), idx_start = 1; end
if isempty(idx_end), idx_end = length(theta); end

t_plot = t_sim(idx_start:idx_end);
theta_plot = theta(idx_start:idx_end);

%% 5. Plotting
figure('Name', 'Time to Double'); hold on; grid on;
plot(t_plot, theta_plot, 'b-', 'LineWidth', 2);

colors = {'r', 'm', 'g', 'k'};
for k = 1:numel(targets)
    xline(t_marks(k), [colors{k} '--'], 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\theta = %.1f deg', targets(k)));
    plot(t_marks(k), theta(idx_marks(k)), 'o', ...
         'MarkerSize', 8, 'MarkerFaceColor', colors{k}, 'MarkerEdgeColor', colors{k});
    text(t_marks(k), theta(idx_marks(k)), ...
         sprintf('  t=%.2fs', t_marks(k)), ...
         'VerticalAlignment', 'bottom', 'Color', colors{k});
end
for k = 2:numel(t_marks)
    dt = t_marks(k) - t_marks(k-1);
    t_mid = 0.5 * (t_marks(k) + t_marks(k-1));
    y_mid = 0.5 * (theta(idx_marks(k)) + theta(idx_marks(k-1)));
    text(t_mid, y_mid, sprintf('\\Delta t = %.3fs', dt), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontWeight', 'bold');
    fprintf('Time from %.1f° to %.1f°: %.3f seconds\n', targets(k-1), targets(k), dt);
end

xlabel('Time (seconds)');
ylabel('Theta Variation (degrees)');
title('Time to Double');
xlim([t_sim(idx_start), t_sim(idx_end)]);
ylim('auto');
legend('show', 'Location', 'northwest');

%% 6. Show Formula & Calculation on Plot
yloc = max(ylim);
xloc = mean(xlim);

formula_str = '$$T_\mathrm{double} = \ln(2) \cdot \sqrt{\frac{I_\mathrm{pitch}}{|M_\alpha|}}$$';
calc_str = sprintf(['$$T_\\mathrm{double} = \\ln(2) \\cdot \\sqrt{\\frac{%.2f}{|%.2f|}} = %.3f\\,\\mathrm{s}$$'], ...
    I_pitch, M_alpha, T_double);

text(xloc, yloc*0.92, formula_str, 'Interpreter', 'latex', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
text(xloc, yloc*0.85, calc_str, 'Interpreter', 'latex', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'BackgroundColor', 'w');

hold off;
