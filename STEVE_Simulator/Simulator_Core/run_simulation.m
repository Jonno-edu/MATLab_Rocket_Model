% run_simulation.m - Main script to run the STEVE rocket simulation

% Initialize parameters
sim_params = initialize_sim_parameters();

% Load Simulink model
modelName = 'STEVE_Simulation';
load_system(modelName);

% Set simulation time
simTime = sim_params.Sim.Time;

% Run simulation
simOut = sim(modelName, 'StopTime', num2str(simTime));

% Generate CSV for Blender (example: export pitch, time, altitude)
if isfield(simOut, 'logsout')
    logsout = simOut.logsout;
    try
        t = logsout.getElement('t').Values.Data;
        pitch = logsout.getElement('pitch').Values.Data;
        altitude = logsout.getElement('altitude').Values.Data;
        csvData = [t, pitch, altitude];
        csvFile = fullfile(sim_params.OutputDataPath, 'trajectory_60fps.csv');
        writematrix(csvData, csvFile);
        fprintf('Blender CSV written to: %s\n', csvFile);
    catch
        disp('Could not find expected signals in logsout for CSV export.');
    end
else
    disp('simOut does not contain logsout. No CSV exported.');
end

% Plot results
plot_simulation_results(simOut); 