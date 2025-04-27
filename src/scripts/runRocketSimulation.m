clear; clc; close all;

% Set up paths
scriptPath = mfilename('fullpath');
[scriptDir, ~, ~] = fileparts(scriptPath);
projectRoot = fullfile(scriptDir, '..', '..');
cd(projectRoot);
disp(['Working directory set to: ', projectRoot]);

% Add paths and locate model
addpath(fullfile(projectRoot, 'src', 'models'));

% Initialize parameters and design control system
initialize_parameters;
ControlSystemDesign;

% Load and simulate the model
modelPath = fullfile(projectRoot, 'src', 'models', 'STEVE_Simulation.slx');
if ~exist(modelPath, 'file')
    error('Cannot find Simulink model at: %s', modelPath);
end

% Load and simulate the model
load_system(modelPath);
out = sim('STEVE_Simulation');

% Extract time vector and data
t = out.tout;
Ve = out.Ve.Data;
Xe = out.Xe.Data;
Vb = out.Vb.Data;
phi = out.phi.Data;
theta = out.theta.Data;
psi = out.psi.Data;
alpha = out.alpha.Data; 

% Extract command data
thetaCmd = out.thetaCmd.Data;
thetadotCmd = out.thetadotCmd.Data;
nozzleCmd = out.nozzleCmd.Data;
nozzleFeedforward = out.nozzleFeedforward.Data;
nozzleCmdTotal = out.nozzleCmdTotal.Data;

% Extract force and moment data
Fwind = out.Fwind.Data;
Mwind = out.Mwind.Data;
Fthrust = out.Fthrust.Data;
Mthrust = out.Mthrust.Data;
Fnet = out.Fnet.Data;
Mnet = out.Mnet.Data;

% Skip the first N frames
N = 1;  % Number of frames to skip
t = t(N+1:end) - Sim.Timestep * N;
Ve = Ve(N+1:end,:);
Xe = Xe(N+1:end,:);
Vb = Vb(N+1:end,:);
phi = phi(N+1:end);
theta = theta(N+1:end);
psi = psi(N+1:end);
alpha = alpha(N+1:end);
thetaCmd = thetaCmd(N+1:end);
thetadotCmd = thetadotCmd(N+1:end);
nozzleCmd = nozzleCmd(N+1:end);
nozzleFeedforward = nozzleFeedforward(N+1:end);
nozzleCmdTotal = nozzleCmdTotal(N+1:end);

% Skip first N frames for forces and moments too
Fwind = Fwind(N+1:end,:);
Mwind = Mwind(N+1:end,:);
Fthrust = Fthrust(N+1:end,:);
Mthrust = Mthrust(N+1:end,:);
Fnet = Fnet(N+1:end,:);
Mnet = Mnet(N+1:end,:);

% Convert earth frame z to make "up" positive
Xe(:,3) = -Xe(:,3);  % Invert z-position (altitude)
Ve(:,3) = -Ve(:,3);  % Invert z-velocity

% Create main figure with tabs
fig = figure('Name', 'Rocket Simulation Results', 'Position', [100, 100, 1200, 700]);
tabgp = uitabgroup(fig);

% Tab 1: Position
tab1 = uitab(tabgp, 'Title', 'Position');

% 3D trajectory
axes('Parent', tab1, 'Position', [0.05, 0.55, 0.3, 0.35]);
plot3(Xe(:,1), Xe(:,2), Xe(:,3), 'b-', 'LineWidth', 2);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory');

% Altitude vs Time (vertical component separate)
axes('Parent', tab1, 'Position', [0.4, 0.55, 0.55, 0.35]);
plot(t, Xe(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude vs Time');

% Ground Track (horizontal plane)
axes('Parent', tab1, 'Position', [0.05, 0.1, 0.3, 0.35]);
plot(Xe(:,1), Xe(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('X (m)'); ylabel('Y (m)');
title('Ground Track (X-Y Plane)');

% Horizontal position components
axes('Parent', tab1, 'Position', [0.4, 0.1, 0.55, 0.35]);
plot(t, Xe(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Xe(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Position (m)');
title('Horizontal Position Components vs Time');
legend('X', 'Y', 'Location', 'best');

% Tab 2: Velocity
tab2 = uitab(tabgp, 'Title', 'Velocity');

% Earth-frame horizontal velocity components
axes('Parent', tab2, 'Position', [0.05, 0.55, 0.4, 0.35]);
plot(t, Ve(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Ve(:,2), 'g-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Earth-Frame Horizontal Velocity');
legend('V_x', 'V_y', 'Location', 'best');

% Earth-frame vertical velocity component (separate)
axes('Parent', tab2, 'Position', [0.55, 0.55, 0.4, 0.35]);
plot(t, Ve(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Earth-Frame Vertical Velocity (Z)');

% Body-frame velocities (leave as is)
axes('Parent', tab2, 'Position', [0.05, 0.1, 0.4, 0.35]);
plot(t, Vb(:,1), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Body-Frame Axial Velocity (X)');

% Body-frame lateral velocity components
axes('Parent', tab2, 'Position', [0.55, 0.1, 0.4, 0.35]);
plot(t, Vb(:,2), 'g-', 'LineWidth', 2); hold on;
plot(t, Vb(:,3), 'b-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Body-Frame Lateral Velocity Components');
legend('V_b_y', 'V_b_z', 'Location', 'best');

% Tab 3: Attitude and Angular Rates
tab3 = uitab(tabgp, 'Title', 'Attitude');

% Extract angular rates
w = out.w.Data;
w = w(N+1:end,:);  % Skip first N frames like other variables

% Extract Nozzle Angle
nozzleAngle = out.NozzleAngle.Data;
nozzleAngle = nozzleAngle(N+1:end); 

% Create a modified angle of attack vector that skips the first 2 values
alpha_modified = alpha(3:end);  % Skip first 2 values
t_alpha = t(3:end);             % Matching time vector for modified alpha

% Adjust layout for 4 plots instead of 3
plot_height = 0.19;             % Reduced height for each plot
vertical_gap = 0.04;            % Reduced gap between plots

% Nozzle Angle (bottom plot)
axes('Parent', tab3, 'Position', [0.1, 0.03, 0.8, plot_height]);
plot(t, nozzleAngle*180/pi, 'b-', 'LineWidth', 2); hold on;
plot(t, nozzleCmd*180/pi/4, 'b--', 'LineWidth', 1.5);
plot(t, nozzleFeedforward*180/pi/4, 'c-.', 'LineWidth', 1.5);
plot(t, nozzleCmdTotal*180/pi/4, 'm:', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Nozzle Deflection Angle');
legend('Actual', 'Command', 'Feedforward', 'Total Cmd', 'Location', 'best');

% Pitch Rate (second from bottom)
axes('Parent', tab3, 'Position', [0.1, 0.03 + plot_height + vertical_gap, 0.8, plot_height]);
plot(t, w(:,2)*180/pi, 'r-', 'LineWidth', 2); hold on;
plot(t, thetadotCmd*180/pi, 'r--', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Pitch Rate (omega_y)');
legend('Actual', 'Command', 'Location', 'best');

% Angle of Attack (third from bottom) - using modified data
axes('Parent', tab3, 'Position', [0.1, 0.03 + 2*(plot_height + vertical_gap), 0.8, plot_height]);
plot(t_alpha, alpha_modified*180/pi, 'm-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Angle of Attack');

% Pitch Angle (top plot)
axes('Parent', tab3, 'Position', [0.1, 0.03 + 3*(plot_height + vertical_gap), 0.8, plot_height]);
plot(t, theta*180/pi, 'g-', 'LineWidth', 2); hold on;
plot(t, thetaCmd*180/pi, 'g--', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle (theta)');
legend('Actual', 'Command', 'Location', 'best');

% Calculate and display key flight metrics
tab4 = uitab(tabgp, 'Title', 'Flight Metrics');

% Calculate metrics
max_altitude = max(Xe(:,3));
[max_speed, max_speed_idx] = max(sqrt(sum(Ve.^2, 2)));
horiz_dist = sqrt(Xe(end,1)^2 + Xe(end,2)^2);
max_horiz_dist = max(sqrt(sum(Xe(:,1:2).^2, 2)));
flight_time = t(end);
[max_accel, max_accel_idx] = max(diff(sqrt(sum(Ve.^2, 2)))./diff(t));
max_aoa = max(abs(alpha)) * 180/pi;

burnout_time = Actuators.Engine.BurnTime;
% Find the index of the time closest to burnout time
[~, burnout_idx] = min(abs(t - burnout_time));
burnout_altitude = Xe(burnout_idx,3);
burnout_speed = sqrt(sum(Ve(burnout_idx,:).^2));
burnout_horiz_dist = sqrt(Xe(burnout_idx,1)^2 + Xe(burnout_idx,2)^2);

% Create text display for metrics
metrics_text = {
    sprintf('Maximum Altitude: %.1f km', max_altitude/1000.0)
    sprintf('Maximum Speed: %.1f m/s (at t=%.1f s)', max_speed, t(max_speed_idx))
    sprintf('Final Distance from Launch Pad: %.1f km', horiz_dist/1000.0)
    sprintf('Maximum Distance from Launch Pad: %.1f km', max_horiz_dist/1000.0)
    sprintf('Total Flight Time: %.1f s', flight_time)
    sprintf('Maximum Acceleration: %.1f m/s²', max_accel)
    sprintf('Maximum Angle of Attack: %.1f degrees', max_aoa)
    sprintf('Burnout Time: %.1f s', burnout_time)
    sprintf('Burnout Altitude: %.1f m', burnout_altitude)
    sprintf('Burnout Speed: %.1f m/s', burnout_speed)
    sprintf('Burnout Horizontal Distance: %.1f m', burnout_horiz_dist)
};

% Display metrics in a clean text box
annotation(tab4, 'textbox', [0.1 0.1 0.8 0.8], ...
    'String', metrics_text, ...
    'FontSize', 14, ...
    'FontName', 'Consolas', ...
    'EdgeColor', 'none', ...
    'VerticalAlignment', 'middle');

% Tab 5: Forces
tab5 = uitab(tabgp, 'Title', 'Forces');

% Axial Forces (Fx)
axes('Parent', tab5, 'Position', [0.05, 0.55, 0.4, 0.35]);
plot(t, Fwind(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, Fthrust(:,1), 'r-', 'LineWidth', 2);
plot(t, Fnet(:,1), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Axial Forces (F_x)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Normal Forces (Fz)
axes('Parent', tab5, 'Position', [0.55, 0.55, 0.4, 0.35]);
plot(t, Fwind(:,3), 'b-', 'LineWidth', 2); hold on;
plot(t, Fthrust(:,3), 'r-', 'LineWidth', 2);
plot(t, Fnet(:,3), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Normal Forces (F_z)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Force Magnitudes
axes('Parent', tab5, 'Position', [0.05, 0.1, 0.4, 0.35]);
plot(t, sqrt(Fwind(:,1).^2 + Fwind(:,3).^2), 'b-', 'LineWidth', 2); hold on;
plot(t, sqrt(Fthrust(:,1).^2 + Fthrust(:,3).^2), 'r-', 'LineWidth', 2);
plot(t, sqrt(Fnet(:,1).^2 + Fnet(:,3).^2), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
title('Force Magnitudes');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Tab 6: Moments
tab6 = uitab(tabgp, 'Title', 'Moments');

% Pitching Moments (My)
axes('Parent', tab6, 'Position', [0.1, 0.55, 0.8, 0.35]);
plot(t, Mwind(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t, Mthrust(:,2), 'r-', 'LineWidth', 2);
plot(t, Mnet(:,2), 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Moment (N·m)');
title('Pitching Moments (M_y)');
legend('Wind', 'Thrust', 'Net', 'Location', 'best');

% Print metrics to command window as well
fprintf('\n=== Key Flight Metrics ===\n');
fprintf('%s\n', metrics_text{:});

% Tab 7: Wind Velocity
tab7 = uitab(tabgp, 'Title', 'Wind Velocity');

% Extract wind velocity data
windVelocity = out.windVelocity.Data;

% Skip the first N frames as with other data
windVelocity = windVelocity(N+1:end,:);

% Plot wind velocity components
axes('Parent', tab7, 'Position', [0.1, 0.55, 0.8, 0.35]);
plot(t, windVelocity(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, windVelocity(:,2), 'g-', 'LineWidth', 2);
plot(t, windVelocity(:,3), 'r-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Wind Velocity Components');
legend('V_x', 'V_y', 'V_z', 'Location', 'best');

% Plot wind speed magnitude
axes('Parent', tab7, 'Position', [0.1, 0.1, 0.8, 0.35]);
windSpeed = sqrt(sum(windVelocity.^2, 2));
plot(t, windSpeed, 'k-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Wind Speed Magnitude');

% Tab 8: Frequency Analysis
tab8 = uitab(tabgp, 'Title', 'Frequency Analysis');

% Perform frequency analysis on pitch rate and nozzle angle
% First, ensure consistent sampling by interpolating if needed
Ts = mean(diff(t));                 % Average time step
fs = 1/Ts;                          % Sampling frequency
disp(['Sampling frequency: ' num2str(fs) ' Hz']);

% Check if sampling frequency is sufficient for high-frequency detection
nyquist_freq = fs/2;
if nyquist_freq < 200  % We want to see up to at least 200 Hz
    warning(['Sampling frequency (' num2str(fs) ' Hz) may be too low to detect 158 Hz oscillations. ' ...
             'Nyquist frequency is ' num2str(nyquist_freq) ' Hz.']);
end
fprintf('Nyquist frequency (max detectable): %.2f Hz\n', nyquist_freq);

% Prepare signals for analysis
pitchRate = w(:,2);                 % Extract pitch rate data
nozzleDeflection = nozzleAngle;     % Nozzle deflection angle

% Compute frequency spectrum for pitch rate
L = length(pitchRate);              % Length of signal
nfft = 2^nextpow2(L);               % Next power of 2 from length
Y = fft(pitchRate, nfft);
f_hz = fs * (0:(nfft/2))/nfft;      % Frequency vector in Hz
f_rads = 2*pi*f_hz;                 % Convert to rad/s
P = abs(Y/nfft);                    % Normalized magnitude
P = P(1:nfft/2+1);                  % Single-sided spectrum
P(2:end-1) = 2*P(2:end-1);          % Account for single-sided spectrum

% Compute frequency spectrum for nozzle angle
Y_nozzle = fft(nozzleDeflection, nfft);
P_nozzle = abs(Y_nozzle/nfft);
P_nozzle = P_nozzle(1:nfft/2+1);
P_nozzle(2:end-1) = 2*P_nozzle(2:end-1);

% Find dominant frequencies
[~, idx_pitch] = findpeaks(P, 'MinPeakHeight', max(P)*0.1, 'SortStr', 'descend');
[~, idx_nozzle] = findpeaks(P_nozzle, 'MinPeakHeight', max(P_nozzle)*0.1, 'SortStr', 'descend');

% Get top frequencies (up to 5) - still in Hz for command line output
top_freq_pitch = f_hz(idx_pitch(1:min(5, length(idx_pitch))));
top_freq_nozzle = f_hz(idx_nozzle(1:min(5, length(idx_nozzle))));

% Look specifically for oscillations around 158 Hz
target_freq = 158;
freq_range = [150, 165];  % Range to look in (Hz)
if max(f_hz) >= freq_range(2)  % Ensure our spectrum reaches this frequency
    % Find indices in this range
    range_indices = find(f_hz >= freq_range(1) & f_hz <= freq_range(2));
    if ~isempty(range_indices)
        % Find local maxima in this range for pitch rate
        [pitch_peaks, pitch_locs] = findpeaks(P(range_indices));
        if ~isempty(pitch_peaks)
            [~, max_idx] = max(pitch_peaks);
            peak_freq_pitch = f_hz(range_indices(pitch_locs(max_idx)));
            fprintf('Pitch rate peak near 158 Hz: %.2f Hz (%.2f rad/s) with magnitude %.4e\n',peak_freq_pitch, peak_freq_pitch*2*pi, P(range_indices(pitch_locs(max_idx))));
        else
            fprintf('No pitch rate peaks found in the 150-165 Hz range\n');
        end
        
        % Find local maxima in this range for nozzle angle
        [nozzle_peaks, nozzle_locs] = findpeaks(P_nozzle(range_indices));
        if ~isempty(nozzle_peaks)
            [~, max_idx] = max(nozzle_peaks);
            peak_freq_nozzle = f_hz(range_indices(nozzle_locs(max_idx)));
            fprintf('Nozzle angle peak near 158 Hz: %.2f Hz (%.2f rad/s) with magnitude %.4e\n',peak_freq_nozzle, peak_freq_nozzle*2*pi, P_nozzle(range_indices(nozzle_locs(max_idx))));
        else
            fprintf('No nozzle angle peaks found in the 150-165 Hz range\n');
        end
    end
else
    fprintf('Frequency spectrum does not reach 158 Hz. Maximum frequency: %.2f Hz (%.2f rad/s)\n',max(f_hz), max(f_hz)*2*pi);
end

% Plot frequency spectrum of pitch rate - Full Range
ax_pitch_full = axes('Parent', tab8, 'Position', [0.1, 0.78, 0.8, 0.18]);
plot(f_rads, P, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
title('Full Frequency Spectrum of Pitch Rate');
% Don't limit xlim here to see the full range

% Plot frequency spectrum of pitch rate - Low Frequency Detail
ax_pitch = axes('Parent', tab8, 'Position', [0.1, 0.53, 0.8, 0.18]);
plot(f_rads, P, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
title('Low Frequency Detail of Pitch Rate');
xlim([0 20*2*pi]);  % Show detail of low frequencies (0-20 Hz in rad/s)

% Plot frequency spectrum of nozzle angle - Full Range
ax_nozzle_full = axes('Parent', tab8, 'Position', [0.1, 0.28, 0.8, 0.18]);
plot(f_rads, P_nozzle, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
title('Full Frequency Spectrum of Nozzle Deflection Angle');
% Don't limit xlim here to see the full range

% Plot frequency spectrum of nozzle angle - Low Frequency Detail
ax_nozzle = axes('Parent', tab8, 'Position', [0.1, 0.03, 0.8, 0.18]);
plot(f_rads, P_nozzle, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
title('Low Frequency Detail of Nozzle Deflection Angle');
xlim([0 20*2*pi]);  % Show detail of low frequencies (0-20 Hz in rad/s)

% If we have sufficient sampling to see 158 Hz, add a plot focused on that region
if nyquist_freq > 200
    % Add a new tab specifically for high-frequency analysis
    tab9 = uitab(tabgp, 'Title', 'High Freq Analysis');
    
    % Plot pitch rate spectrum around 158 Hz
    ax_pitch_high = axes('Parent', tab9, 'Position', [0.1, 0.55, 0.8, 0.35]);
    plot(f_rads, P, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Frequency (rad/s)');
    ylabel('Magnitude');
    title('Pitch Rate Spectrum - High Frequency Region');
    xlim([100*2*pi 200*2*pi]);  % 100-200 Hz range in rad/s
    
    % Plot nozzle angle spectrum around 158 Hz
    ax_nozzle_high = axes('Parent', tab9, 'Position', [0.1, 0.1, 0.8, 0.35]);
    plot(f_rads, P_nozzle, 'g-', 'LineWidth', 1.5);
    grid on;
    xlabel('Frequency (rad/s)');
    ylabel('Magnitude');
    title('Nozzle Angle Spectrum - High Frequency Region');
    xlim([100*2*pi 200*2*pi]);  % 100-200 Hz range in rad/s

    % Add a note showing the conversion between Hz and rad/s for the region of interest
    annotation(tab9, 'textbox', [0.1, 0.01, 0.8, 0.05], ...
        'String', sprintf('Note: 158 Hz ≈ %.1f rad/s', 158*2*pi), ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 10);
end

% Print dominant frequencies to command window
fprintf('\n=== Dominant Oscillation Frequencies ===\n');
fprintf('Pitch Rate Dominant Frequencies: %s Hz (%s rad/s)\n', ...
    strjoin(arrayfun(@(x) sprintf('%.2f', x), top_freq_pitch, 'UniformOutput', false), ', '), ...
    strjoin(arrayfun(@(x) sprintf('%.2f', x*2*pi), top_freq_pitch, 'UniformOutput', false), ', '));
fprintf('Nozzle Angle Dominant Frequencies: %s Hz (%s rad/s)\n', ...
    strjoin(arrayfun(@(x) sprintf('%.2f', x), top_freq_nozzle, 'UniformOutput', false), ', '), ...
    strjoin(arrayfun(@(x) sprintf('%.2f', x*2*pi), top_freq_nozzle, 'UniformOutput', false), ', '));

% --- START: Updated Data Export Section ---

% Save trajectory data for Python visualization
sim_fps = 1/Sim.Timestep;          % Original simulation FPS
target_fps = 100;                   % Desired output FPS
downsample_factor = round(sim_fps/target_fps);

% Downsample data for 100 FPS
sampled_indices = 1:downsample_factor:length(t);

% Interpolate CG data (Assuming COM_X is CG distance from AFT end)
sampled_times = t(sampled_indices);
cg_from_aft_m = interp1(MassData.Time, MassData.COM_X, sampled_times, 'linear', 'extrap');

% Calculate CG position relative to the NOSE in body frame
% Assumes body X-axis points forward from the nose.
rocket_length = RocketAeroPhysical.Length; % Ensure this variable exists
cg_body_x_m = rocket_length - cg_from_aft_m; % Distance from nose along body X

% Assuming CG offset is only along the body X-axis in this 3DOF sim
cg_body_y_m = zeros(size(cg_body_x_m));
cg_body_z_m = zeros(size(cg_body_x_m));

% Get sampled position, pitch, and nozzle angle
pos_xyz_m = Xe(sampled_indices,:);         % World frame [X, Y, Z_up]
pitch_rad = theta(sampled_indices);        % Pitch angle (radians)
nozzle_angle_rad = nozzleAngle(sampled_indices); % Nozzle angle (radians)

% Combine data for export (Focus on data needed for Blender animation)
trajectory_data = [t(sampled_indices), ...
                   pos_xyz_m, ...             % World Pos X, Y, Z
                   pitch_rad, ...             % Body Pitch
                   cg_body_x_m, ...           % CG X relative to nose
                   cg_body_y_m, ...           % CG Y relative to nose
                   cg_body_z_m, ...           % CG Z relative to nose
                   nozzle_angle_rad];         % Nozzle Angle

% Create table with explicit headers for Blender script
trajectory_table = array2table(trajectory_data, ...
    'VariableNames', {'time_s', 'pos_x_m', 'pos_y_m', 'pos_z_m', ...
                     'pitch_rad', ...
                     'cg_body_x_m', 'cg_body_y_m', 'cg_body_z_m', ...
                     'nozzle_angle_rad'});

% Get the root directory path and construct absolute file path
scriptPath = mfilename('fullpath');
[scriptDir, ~, ~] = fileparts(scriptPath);
projectRoot = fullfile(scriptDir, '..', '..');
resultsFolder = fullfile(projectRoot, 'data', 'results');

% Create results directory if it doesn't exist
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
    fprintf('Created results directory: %s\n', resultsFolder);
end

% Construct full file path and save CSV
resultsFilePath = fullfile(resultsFolder, 'rocket_trajectory_100fps.csv');
writetable(trajectory_table, resultsFilePath);
fprintf('\nExported trajectory data to %s\n', resultsFilePath);

% --- END: Updated Data Export Section ---

% Tab 10: Controller Information
controllerTab = uitab(tabgp, 'Title', 'Controller Info');

% Collect controller parameters
inner_kp = inner_Kp;  % Inner loop proportional gain
inner_ki = inner_Ki;  % Inner loop integral gain
outer_kp = outer_Kp;  % Outer loop proportional gain
outer_ki = outer_Ki;  % Outer loop integral gain

% Designed actuator parameters (used for controller design)
designed_actuator_type = 'Second Order';
designed_actuator_natural_freq = 1500;  % Natural frequency (rad/s)
designed_actuator_damping_ratio = 0.707;  % Damping ratio

% Actual actuator parameters (from initialization)
actual_actuator_type = 'Second Order';
actual_actuator_natural_freq = Actuators.Nozzle.NaturalFreq;  % Natural frequency (rad/s)
actual_actuator_damping_ratio = Actuators.Nozzle.DampingRatio;  % Damping ratio

inner_bandwidth = target_bw_inner_hint;  % Inner loop bandwidth (rad/s)
inner_phase_margin = target_pm_inner;    % Inner loop phase margin (degrees)
outer_bandwidth = target_bw_outer_hint;  % Outer loop bandwidth (rad/s)
outer_phase_margin = target_pm_outer;    % Outer loop phase margin (degrees)

% Create text display for controller parameters
controller_text = {
    'CONTROLLER PARAMETERS', 
    '=====================', 
    '',
    'Inner Loop (Pitch Rate) Controller:',
    sprintf('  Kp = %.4f', inner_kp),
    sprintf('  Ki = %.4f', inner_ki),
    sprintf('  Bandwidth = %.3f rad/s (%.3f Hz)', inner_bandwidth, inner_bandwidth/(2*pi)),
    sprintf('  Phase Margin = %.2f°', inner_phase_margin),
    '',
    'Outer Loop (Pitch Angle) Controller:',
    sprintf('  Kp = %.4f', outer_kp),
    sprintf('  Ki = %.4f', outer_ki),
    sprintf('  Bandwidth = %.3f rad/s (%.3f Hz)', outer_bandwidth, outer_bandwidth/(2*pi)),
    sprintf('  Phase Margin = %.2f°', outer_phase_margin),
    '',
    'Designed Actuator Specifications (Controller Design):',
    sprintf('  Type: %s', designed_actuator_type),
    sprintf('  Natural Frequency = %.3f rad/s (%.3f Hz)', designed_actuator_natural_freq, designed_actuator_natural_freq/(2*pi)),
    sprintf('  Damping Ratio = %.3f', designed_actuator_damping_ratio),
    '',
    'Actual Actuator Specifications:',
    sprintf('  Type: %s', actual_actuator_type),
    sprintf('  Natural Frequency = %.3f rad/s (%.3f Hz)', actual_actuator_natural_freq, actual_actuator_natural_freq/(2*pi)),
    sprintf('  Damping Ratio = %.3f', actual_actuator_damping_ratio)
};

% Display controller info in a clean text box
annotation(controllerTab, 'textbox', [0.1, 0.1, 0.8, 0.8], ...
    'String', controller_text, ...
    'FontSize', 12, ...
    'FontName', 'Consolas', ...
    'EdgeColor', 'none', ...
    'VerticalAlignment', 'top');

% Tab 11: Combined Attitude and Controller Info
combinedTab = uitab(tabgp, 'Title', 'Attitude + Controller Info');

%g Create a layout with plots on the left and controller info on the right
% Attitude plots (left side)
plotPanel = uipanel('Parent', combinedTab, 'Position', [0.05, 0.05, 0.6, 0.9], 'Title', 'Attitude Plots');

% Nozzle Angle (bottom plot)
axes('Parent', plotPanel, 'Position', [0.1, 0.05, 0.8, 0.2]);
plot(t, nozzleAngle*180/pi, 'b-', 'LineWidth', 2); hold on;
plot(t, nozzleCmd*180/pi/4, 'b--', 'LineWidth', 1.5);
plot(t, nozzleFeedforward*180/pi/4, 'c-.', 'LineWidth', 1.5);
plot(t, nozzleCmdTotal*180/pi/4, 'm:', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Nozzle Deflection Angle');
legend('Actual', 'Command', 'Feedforward', 'Total Cmd', 'Location', 'best');

% Pitch Rate (second from bottom)
axes('Parent', plotPanel, 'Position', [0.1, 0.3, 0.8, 0.2]);
plot(t, w(:,2)*180/pi, 'r-', 'LineWidth', 2); hold on;
plot(t, thetadotCmd*180/pi, 'r--', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Pitch Rate (omega_y)');
legend('Actual', 'Command', 'Location', 'best');

% Angle of Attack (third from bottom)
axes('Parent', plotPanel, 'Position', [0.1, 0.55, 0.8, 0.2]);
plot(t_alpha, alpha_modified*180/pi, 'm-', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Angle of Attack');

% Pitch Angle (top plot)
axes('Parent', plotPanel, 'Position', [0.1, 0.8, 0.8, 0.2]);
plot(t, theta*180/pi, 'g-', 'LineWidth', 2); hold on;
plot(t, thetaCmd*180/pi, 'g--', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle (theta)');
legend('Actual', 'Command', 'Location', 'best');

% Controller information (right side)
controllerPanel = uipanel('Parent', combinedTab, 'Position', [0.7, 0.05, 0.25, 0.9], 'Title', 'Controller Information');

% Display controller info in a clean text box
annotation(controllerPanel, 'textbox', [0.05, 0.05, 0.9, 0.9], ...
    'String', controller_text, ...
    'FontSize', 10, ...
    'FontName', 'Consolas', ...
    'EdgeColor', 'none', ...
    'VerticalAlignment', 'top');


% --- Tab: Pitch & Nozzle Analysis ---
tabPitch = uitab(tabgp, 'Title', 'Pitch & Nozzle');
tlo = tiledlayout(tabPitch, 3, 1, 'TileSpacing','compact','Padding','compact');

% 1) Pitch angle and command
ax1 = nexttile(tlo, 1);
plot(ax1, t, theta*180/pi,     'g-',  'LineWidth', 2); hold(ax1,'on');
plot(ax1, t, thetaCmd*180/pi, 'g--', 'LineWidth', 1.5); hold(ax1,'off');
grid(ax1,'on');
xlabel(ax1,'Time (s)');
ylabel(ax1,'Angle (°)');
title(ax1,'Pitch Angle vs Command');
legend(ax1,'Actual','Command','Location','best');


% 2) Nozzle angle vs time
ax2 = nexttile(tlo, 2);
plot(ax2, t, nozzleAngle*180/pi, 'b-', 'LineWidth', 2);
grid(ax2,'on');
xlabel(ax2,'Time (s)');
ylabel(ax2,'Angle (°)');
title(ax2,'Nozzle Deflection Angle');


% Annotate initial pitch rate
pr0 = Initial.Conditions.pitchRate;  % [deg/s]
yl = ylim(ax1);
text(ax1, t(1), yl(2), sprintf('Pitch Rate to vertical = %.1f°/s', pr0), ...
     'VerticalAlignment','top','FontSize',10);
% 3) Nozzle angle box‐and‐whisker with angle on the x-axis
ax3 = nexttile(tlo, 3);
b = boxchart(ax3, nozzleAngle*180/pi, 'Orientation','horizontal');  % Horizontal box chart[3]
grid(ax3, 'on');
xlabel(ax3, 'Angle (°)');
title(ax3, 'Nozzle Angle Distribution');
