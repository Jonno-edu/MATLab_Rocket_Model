
% --- FFT Analysis Script for Your Specific Simulation Output ---

fprintf('\n--- Performing Frequency Analysis on Simulation Output ---\n');

% 1. Pre-computation Check
% Ensure the simOut variable exists in the workspace
if ~exist('simOut', 'var')
    error('Could not find the "simOut" variable in the workspace. Please run your simulation first.');
end

% 2. Extract Pitch Rate Data from the Nested Bus Structure
try
    % Get the top-level logsout object
    logsout_data = simOut.logsout;
    
    % Navigate into the 'PlantData' bus
    plant_data_obj = logsout_data.getElement('PlantData');
    plant_data_struct = plant_data_obj.Values;
    
    % Access the 'w' timeseries object (angular rates)
    w_timeseries = plant_data_struct.w;
    
    % Extract the raw data matrix and the time vector
    w_data_matrix = w_timeseries.Data; % This is an N x 3 matrix [p, q, r]
    t = w_timeseries.Time;
    
    % Extract the pitch rate 'q' (the second column)
    pitch_rate_q = w_data_matrix(:, 2);
    
    fprintf('Successfully extracted pitch rate data ("w(2)").\n');
    
catch ME
    error('Failed to extract pitch rate data. Check the signal path in your logsout structure. Error was: %s', ME.message);
end

% 3. Perform the Fast Fourier Transform (FFT)
fs = 1 / (t(2) - t(1));      % Calculate the sample rate
L = length(pitch_rate_q);   % Length of the signal
Y = fft(pitch_rate_q);      % Compute the FFT

% Compute the two-sided spectrum P2, then the single-sided spectrum P1
P2 = abs(Y / L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain vector f
f = fs * (0:(L/2)) / L;

% 4. Find the Peak Frequency (The Oscillation)
% We ignore the DC component at f=0 by starting the search at index 2
[peak_amplitude, peak_index] = max(P1(2:end));
% Add 1 to the index to account for ignoring the DC component
resonant_frequency_hz = f(peak_index + 1); 

fprintf('------------------------------------------------------\n');
fprintf('Dominant resonant frequency identified at: %.2f Hz\n', resonant_frequency_hz);
fprintf('------------------------------------------------------\n');


% 5. Plot the Frequency Spectrum for Visual Confirmation
figure;
plot(f, P1) 
title('Frequency Spectrum of Pitch Rate (w(2))')
xlabel('Frequency (Hz)')
ylabel('Amplitude')
grid on;
hold on;
% Mark the identified peak
plot(resonant_frequency_hz, peak_amplitude, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
legend('Spectrum', sprintf('Peak at %.2f Hz', resonant_frequency_hz));
hold off;

