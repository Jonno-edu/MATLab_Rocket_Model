%% Full Flight Time to Double Calculation and Plotting
% This script calculates the analytical time-to-double and plots the
% results alongside each of the key contributing flight parameters.

%% 1. Data Extraction
% Ensure the 'simOut' variable is in the workspace.
if ~exist('simOut', 'var')
    error('The "simOut" variable is not in the workspace. Please run your simulation first.');
end
logsout_data = simOut.logsout;

% Extract all necessary signals from the simulation logs
t_original = logsout_data.getElement('theta_variation_deg').Values.Time;
V_original = logsout_data.getElement('airspeed').Values.Data;
I_original = logsout_data.getElement('I').Values.Data; % Assumes 3x3xN
Cma_original = logsout_data.getElement('Cmalpha').Values.Data;
L_arm_original = logsout_data.getElement('CG_X').Values.Data;
S_original = logsout_data.getElement('Lref').Values.Data;
rho_original = logsout_data.getElement('ENV').Values.AirDensity.Data;

%% 2. Data Preparation (Minimal)
% Ensure all data are column vectors to prevent dimension errors
t_original = t_original(:);
V_original = V_original(:);
rho_original = rho_original(:);
Cma_original = Cma_original(:);
L_arm_original = L_arm_original(:);
S_original = S_original(:);
% Extract I_yy and convert from a 1x1xN array to a column vector
I_pitch_original = squeeze(I_original(2, 2, :));

% Synchronize vector lengths to the shortest vector to avoid interp errors
minLength = min(cellfun(@length, {t_original, V_original, rho_original, Cma_original, L_arm_original, S_original, I_pitch_original}));
t_original = t_original(1:minLength);
V_original = V_original(1:minLength);
rho_original = rho_original(1:minLength);
Cma_original = Cma_original(1:minLength);
L_arm_original = L_arm_original(1:minLength);
S_original = S_original(1:minLength);
I_pitch_original = I_pitch_original(1:minLength);

%% 3. Resample Data at 0.1s Intervals
% Define the new time vector with a 0.1s step
t_resampled = (t_original(1):0.1:t_original(end))';

% Interpolate all variables onto the new time vector
V_resampled = interp1(t_original, V_original, t_resampled, 'linear');
rho_resampled = interp1(t_original, rho_original, t_resampled, 'linear');
Cma_resampled = interp1(t_original, Cma_original, t_resampled, 'linear');
L_arm_resampled = interp1(t_original, L_arm_original, t_resampled, 'linear');
S_resampled = interp1(t_original, S_original, t_resampled, 'linear');
I_pitch_resampled = interp1(t_original, I_pitch_original, t_resampled, 'linear');

%% 4. Perform Calculation
% Calculate dynamic pressure
q_dyn_resampled = 0.5 * rho_resampled .* V_resampled.^2;
% Calculate the pitching moment derivative (M_alpha)
M_alpha_t = q_dyn_resampled .* S_resampled .* L_arm_resampled .* Cma_resampled;
% Calculate time-to-double
T_double_t = log(2) * sqrt(I_pitch_resampled ./ M_alpha_t);
T_double_t(M_alpha_t <= 0 | I_pitch_resampled <= 0) = NaN; % Mark invalid results as NaN

%% 5. Plotting Results
figure('Name', 'Time to Double Full Flight Analysis', 'Position', [100, 100, 800, 900]);

% Subplot 1: Final Time to Double
ax1 = subplot(5, 1, 1);
plot(t_resampled, T_double_t, 'b-', 'LineWidth', 1.5);
title('Analytical Time to Double (T_d)');
ylabel('T_{double} (s)');
grid on;

% Find and annotate minimum T_double
[minT, minIdx] = min(T_double_t);
minT_time = t_resampled(minIdx);
hold on;
plot(minT_time, minT, 'ro', 'MarkerSize', 7, 'LineWidth', 2);
text(minT_time, minT, sprintf('  Min: %.3f s @ %.2f s', minT, minT_time), ...
    'VerticalAlignment', 'bottom', 'Color', 'red', 'FontSize', 10);
hold off;

ylim([0 1]); % Scale Y-axis to a max of 1s


% Subplot 2: Pitching Moment Derivative
ax2 = subplot(5, 1, 2);
plot(t_resampled, M_alpha_t, 'r-', 'LineWidth', 1.5);
title('Pitching Moment Derivative (M_{\alpha})');
ylabel('Nm/rad');
grid on;

% Subplot 3: Pitch Inertia
ax3 = subplot(5, 1, 3);
plot(t_resampled, I_pitch_resampled, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
title('Pitch Inertia (I_y)');
ylabel('kg*m^2');
grid on;

% Subplot 4: Cm_alpha
ax4 = subplot(5, 1, 4);
plot(t_resampled, Cma_resampled, 'g-', 'LineWidth', 1.5);
title('Pitching Moment Coefficient Derivative (Cm_{\alpha})');
ylabel('1/rad');
grid on;

% Subplot 5: Dynamic Pressure and Airspeed
ax5 = subplot(5, 1, 5);
yyaxis left;
plot(t_resampled, q_dyn_resampled, 'm-', 'LineWidth', 1.5);
ylabel('Dyn. Pressure (Pa)');
yyaxis right;
plot(t_resampled, V_resampled, 'k--', 'LineWidth', 1.5);
ylabel('Airspeed (m/s)');
title('Dynamic Pressure and Airspeed');
xlabel('Time (seconds)');
grid on;

% Link all x-axes for synchronized zooming and panning
linkaxes([ax1, ax2, ax3, ax4, ax5], 'x');
xlim(ax1, [t_resampled(1), t_resampled(end)]);

fprintf('\nScript finished. Full flight analysis plot has been generated.\n');
