% Define user parameters
wn = 95;
zeta = 0.7;

% 1. Mathematical Model of Second-Order Actuator
num_model = wn^2;
den_model = [1, 2*zeta*wn, wn^2];
sys_model = tf(num_model, den_model); % Transfer function of the model
disp('--- Mathematical Model Parameters ---');
disp(['Natural Frequency (wn_model): ', num2str(wn), ' rad/s']);
disp(['Damping Ratio (zeta_model): ', num2str(zeta)]);
disp(' ');

% 2. System Identification from Simulink Model
% Simulate the Simulink model to get input-output data
disp('Running Simulink model "second_order_sine_sweep.slx"...');
simOut = sim('second_order_sine_sweep.slx'); % Capture sim output
disp('Simulink model simulation complete.');
disp(' ');

% Extract input and output data from simulation
try
    % Get the timeseries object for the input signal (chirp)
    u_sim_ts = simOut.get('sinChirp'); 
    
    % Get the timeseries object for the Servo's Y_nozzle_angle output
    % This is now logged via the "To Workspace" block you labeled 'out.servo_A_actual'.
    % We assume the "Variable name" property inside that block is 'servo_A_actual'.
    y_output_ts = simOut.get('servo_A_actual'); 

    t = u_sim_ts.Time;      % Time vector from the input signal
    u = u_sim_ts.Data;      % Actuator demand (input for identification)
    
    % This 'y' is the output used for system identification (tfest)
    y = y_output_ts.Data;   % Output from servo (Y_nozzle_angle)
    
    % This 'y_actuator_for_comparison' is the signal we will compare against in plots.
    % In this case, it's the same Y_nozzle_angle.
    y_actuator_for_comparison = y_output_ts.Data;

    % Ensure u, y, t, and y_actuator_for_comparison are column vectors
    u = u(:);
    y = y(:);
    t = t(:);
    y_actuator_for_comparison = y_actuator_for_comparison(:);

catch ME
    disp(' ');
    disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
    disp('!!! ERROR EXTRACTING DATA FROM SIMULINK (simOut) !!!');
    disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
    disp('Please ensure signal logging in your Simulink model ("second_order_sine_sweep.slx") is configured correctly.');
    disp('The script expects "To Workspace" blocks with specific "Variable name" properties set:');
    disp('  1. For the input chirp signal: "sinChirp"');
    disp('  2. For the Servo Y_nozzle_angle output: "servo_A_actual" (all lowercase "s")');
    disp(' ');
    disp(['MATLAB Error Message: ', ME.message]);
    if contains(ME.message, 'Unrecognized field name') || contains(ME.message, 'InvalidSignalName')
        disp('This usually means the "Variable name" inside one of your "To Workspace" blocks in Simulink');
        disp('does not EXACTLY match (case-sensitive) what the script is trying to access.');
        disp('Based on your image, the block connected to Y_nozzle_angle is labeled "out.servo_A_actual".');
        disp('Double-check its "Variable name" property is set to "servo_A_actual".');
    end
    disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
    disp(' ');
    return; % Exit if data extraction fails
end

% Estimate the transfer function using system identification
np = 2; % Number of poles
nz = 0; % Number of zeros (assuming a standard second-order system)
Ts_ident = t(2)-t(1); % Sample time for identification
ident_data = iddata(y, u, Ts_ident); 
sys_identified = tfest(ident_data, np, nz); % Requires System Identification Toolbox

disp('--- Identified System Parameters (from A_demand -> Y_nozzle_angle) ---');
[wn_identified_vec, zeta_identified_vec, poles_identified] = damp(sys_identified);

if ~isempty(wn_identified_vec)
    wn_identified = wn_identified_vec(1);
    zeta_identified = zeta_identified_vec(1);
    disp(['Identified Natural Frequency (wn_identified): ', num2str(wn_identified), ' rad/s']);
    disp(['Identified Damping Ratio (zeta_identified): ', num2str(zeta_identified)]);
else
    disp('Could not determine natural frequency and damping ratio for the identified system.');
    wn_identified = NaN;
    zeta_identified = NaN;
end
disp(' ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Compare to Servo Block's Y_nozzle_angle Output (logged as 'servo_A_actual')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--- Comparing Models to Servo Y_nozzle_angle Output (logged as "servo_A_actual") ---');

% Simulate the hand-calculated model with the chirp input
output_model_simulated = lsim(sys_model, u, t);

% Simulate the system identified (from u -> y) with the chirp input
output_identified_simulated = lsim(sys_identified, u, t);

% Plot the results
figure;
plot(t, y_actuator_for_comparison, 'k', 'LineWidth', 1.5, 'DisplayName', 'Simulink Servo Output (Y_nozzle_angle)');
hold on;
plot(t, output_model_simulated, 'b--', 'DisplayName', 'Hand-Calculated Model Response');
plot(t, output_identified_simulated, 'r:', 'DisplayName', 'Identified System Response'); % Changed linestyle for clarity
hold off;
xlabel('Time (s)');
ylabel('Output Amplitude');
title('Comparison: Simulink Servo Output vs. Model Responses (Time Domain)');
legend('Location', 'best');
grid on;


% Calculate RMSE and NRMSE (Normalized RMSE) directly
rmse_model_vs_actuator = sqrt(mean((output_model_simulated - y_actuator_for_comparison).^2)); % Root Mean Squared Error
rmse_identified_vs_actuator = sqrt(mean((output_identified_simulated - y_actuator_for_comparison).^2));

% NRMSE - Normalizing by the range of the observed data (y_actuator_for_comparison)
range_actuator = max(y_actuator_for_comparison) - min(y_actuator_for_comparison);
if range_actuator > 0  % To avoid division by zero
    nrmse_model_vs_actuator = rmse_model_vs_actuator / range_actuator;
    nrmse_identified_vs_actuator = rmse_identified_vs_actuator / range_actuator;
else
    nrmse_model_vs_actuator = NaN; % Handle the case where the range is zero
    nrmse_identified_vs_actuator = NaN;
end

fprintf('RMSE (Hand-Calculated Model vs. Simulink Servo Output): %.4f\n', rmse_model_vs_actuator);
fprintf('NRMSE (Hand-Calculated Model vs. Simulink Servo Output): %.4f\n', nrmse_model_vs_actuator);
fprintf('RMSE (Identified System vs. Simulink Servo Output): %.4f\n', rmse_identified_vs_actuator);
fprintf('NRMSE (Identified System vs. Simulink Servo Output): %.4f\n', nrmse_identified_vs_actuator);

disp(' ');

% 4. Plotting the Step Response of Models
figure;
h_step = stepplot(sys_model, 'b', sys_identified, 'r--');
title('Step Response Comparison (Hand-Calculated vs. Identified Model)');
legend('Hand-Calculated Model', 'Identified Model');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
ax_step = gca;
ylim(ax_step, 'auto');

% 5. Plotting the Bode Plot of Models
figure;
h_bode = bodeplot(sys_model, 'b', sys_identified, 'r--');
title('Bode Plot Comparison (Hand-Calculated vs. Identified Model)');
legend('Hand-Calculated Model', 'Identified Model');
grid on;

% 6. Quantitative Comparison (Parameters of Hand-Calculated vs. Identified Model)
disp('--- Quantitative Comparison (Hand-Calculated Model vs. Identified Model) ---');

if ~isnan(wn_identified) && ~isnan(zeta_identified) && wn ~= 0 && zeta ~=0 
    wn_diff_percent = abs((wn - wn_identified) / wn) * 100;
    zeta_diff_percent = abs((zeta - zeta_identified) / zeta) * 100;
    disp('Parameter Differences:');
    fprintf('Natural Frequency Difference: %.2f%%\n', wn_diff_percent);
    fprintf('Damping Ratio Difference: %.2f%%\n', zeta_diff_percent);
else
    disp('Cannot calculate parameter differences (original or identified wn/zeta might be zero or NaN).');
end
disp(' ');

% Step Response MAPE (Hand-Calculated Model vs. Identified Model)
[y_model_step, t_model_step] = step(sys_model);
[y_identified_step, t_identified_step] = step(sys_identified, t_model_step(end)); 

if length(t_model_step) ~= length(t_identified_step) || any(t_model_step ~= t_identified_step)
    y_identified_step_interp = interp1(t_identified_step, y_identified_step, t_model_step, 'linear', 'extrap');
else
    y_identified_step_interp = y_identified_step;
end

epsilon = 1e-6; 
valid_indices_step = abs(y_model_step) > epsilon;
if any(valid_indices_step)
    step_mape = mean(abs((y_model_step(valid_indices_step) - y_identified_step_interp(valid_indices_step)) ./ y_model_step(valid_indices_step))) * 100;
    fprintf('Step Response Mean Absolute Percentage Error (MAPE) between models: %.2f%%\n', step_mape);
else
    disp('Cannot calculate Step Response MAPE between models (model step response is near zero everywhere).');
end
disp(' ');

% Bode Plot Comparison (Hand-Calculated Model vs. Identified Model)
[mag_model, phase_model, wout_model] = bode(sys_model);
[mag_identified, phase_identified, wout_identified] = bode(sys_identified, wout_model); 

mag_model = squeeze(mag_model);
phase_model = squeeze(phase_model);
mag_identified = squeeze(mag_identified);
phase_identified = squeeze(phase_identified);

valid_indices_mag = abs(mag_model) > epsilon;
if any(valid_indices_mag)
    mag_mape = mean(abs((mag_model(valid_indices_mag) - mag_identified(valid_indices_mag)) ./ mag_model(valid_indices_mag))) * 100;
    fprintf('Bode Magnitude Mean Absolute Percentage Error (MAPE) between models: %.2f%%\n', mag_mape);
else
    disp('Cannot calculate Bode Magnitude MAPE between models (model magnitude response is near zero everywhere).');
end

phase_mae = mean(abs(phase_model - phase_identified));
fprintf('Bode Phase Mean Absolute Error (MAE) between models: %.2f degrees\n', phase_mae);
disp(' ');
disp('Note: MAPE can be sensitive if the reference values are close to zero.');
disp('MAE for phase is given in degrees.');
