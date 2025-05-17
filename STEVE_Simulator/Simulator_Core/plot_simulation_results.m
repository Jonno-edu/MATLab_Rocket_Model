function plot_simulation_results(simOut)
% plot_simulation_results - Plots the omega signal from the simulation output for testing.
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')

figure;
try
    logsout = simOut.logsout;
    % Check if omega is present in logsout
    names = logsout.getElementNames;
    if any(strcmp(names, 'omega'))
        omega = logsout.getElement('omega').Values.Data;
        t = logsout.getElement('omega').Values.Time;
        plot(t, omega);
        xlabel('Time (s)'); ylabel('\omega');
        title('Logged omega signal');
        grid on;
    else
        disp('The omega signal is not found in logsout. Available signals:');
        disp(names);
    end
catch
    disp('simOut does not contain logsout. Please check signal logging in Simulink.');
end
end 