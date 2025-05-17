function inspect_logsout_deep(simOut)
% inspect_logsout_deep - Iterates through signals in logsout, inspects
%                        nested bus structures, and prints parsing instructions.
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')

% --- Initial Checks ---
if ~exist('simOut', 'var') || isempty(simOut)
    error('inspect_logsout_deep: simOut variable not provided or is empty.');
end
if ~isprop(simOut, 'logsout') || isempty(simOut.logsout)
    error('inspect_logsout_deep: logsout not found in simOut or is empty.');
end

logsout = simOut.logsout;
if ~isa(logsout, 'Simulink.SimulationData.Dataset')
    error('inspect_logsout_deep: logsout is not a Simulink.SimulationData.Dataset object.');
end

fprintf('--- Iterating through signals in logsout (Deep Inspection) ---\n\n');

% Get the names of all top-level signals in the logsout Dataset
topLevelSignalNames = logsout.getElementNames();

% Loop through each top-level signal
for i = 1:length(topLevelSignalNames)
    currentSignalName = topLevelSignalNames{i};
    
    if isempty(currentSignalName)
        fprintf('Top-level signal: [Unnamed Signal]\n');
    else
        fprintf('Top-level signal: %s\n', currentSignalName);
    end

    try
        signalObject = logsout.getElement(currentSignalName);
        signalValues = signalObject.Values;
        inspect_signal_recursively(signalValues, '  ', 2); % Initial indent, max depth 2 for sub-buses
    catch ME
        fprintf('  Error processing signal "%s" during inspection: %s\n', currentSignalName, ME.message);
    end
    fprintf('\n'); 
end

fprintf('--- End of signal iteration ---\n');

% --- Instructions for Programmatically Accessing Logged Signal Data ---
fprintf('\n\n--- How to Programmatically Access Logged Signal Data (Based on Above Structure) ---\n');
fprintf('The simulation output `simOut` contains logged signals in `simOut.logsout`.\n');
fprintf('To work with this data, first get the logsout object: `logsout_data = simOut.logsout;`\n\n');

fprintf('1. Accessing Top-Level Signals:\n');
fprintf('   Use `getElement` method: `signal_obj = logsout_data.getElement(''TopLevelSignalName'');`\n');
fprintf('   Example: `plant_data_obj = logsout_data.getElement(''PlantData'');`\n\n');

fprintf('2. Getting Actual Data from a Signal Object:\n');
fprintf('   The data resides in the `.Values` property of the signal object.\n');
fprintf('   - If it''s a SINGLE signal (not a bus, typically a `timeseries` object):\n');
fprintf('     `timeseries_data = signal_obj.Values;`\n');
fprintf('     `actual_values = timeseries_data.Data;` (This can be scalar, vector, or matrix per time step)\n');
fprintf('     `time_vector = timeseries_data.Time;`\n');
fprintf('     Example: `F_total_ts = logsout_data.getElement(''F_total'').Values; F_values = F_total_ts.Data;`\n\n');

fprintf('3. Accessing Signals WITHIN a BUS (Nested Structures):\n');
fprintf('   If `signal_obj.Values` is a struct (as indicated by "Type: Bus (struct)" above), \n');
fprintf('   you access its elements using dot notation.\n\n');

fprintf('   Level 1 Deep (e.g., a signal directly within ''PlantData''):\n');
fprintf('     `plant_data_struct = logsout_data.getElement(''PlantData'').Values;`\n');
fprintf('     `alpha_timeseries = plant_data_struct.alpha;` (If ''alpha'' is a field in PlantData)\n');
fprintf('     `alpha_values = alpha_timeseries.Data;`\n\n');

fprintf('   Level 2 Deep (e.g., ''theta'' within ''body_angles'' which is within ''PlantData''):\n');
fprintf('     `plant_data_struct = logsout_data.getElement(''PlantData'').Values;`\n');
fprintf('     `body_angles_struct = plant_data_struct.body_angles;`\n');
fprintf('     `pitch_timeseries = body_angles_struct.theta;`\n');
fprintf('     `pitch_values = pitch_timeseries.Data;`\n\n');

fprintf('   Level 3+ Deep (Continue the pattern):\n');
fprintf('     `level1_bus = top_bus_struct.Level1BusName;`\n');
fprintf('     `level2_bus = level1_bus.Level2BusName;`\n');
fprintf('     `target_signal_ts = level2_bus.SignalNameInLevel2Bus;`\n\n');

fprintf('4. Handling Matrix Signals within a Timeseries:\n');
fprintf('   If `timeseries_data.Data` is a matrix (e.g., for vector signals like angular rates [p;q;r]),\n');
fprintf('   it''s often `N x M`, where N is number of time points, M is number of signal components.\n');
fprintf('   Example: If `plant_data_struct.w.Data` contains [p, q, r] as columns:\n');
fprintf('     `angular_rates_matrix = plant_data_struct.w.Data;`\n');
fprintf('     `pitch_rate_q_vector = angular_rates_matrix(:, 2);` (Second column for q)\n');
fprintf('     `roll_rate_p_vector = angular_rates_matrix(:, 1);` (First column for p)\n\n');

fprintf('5. Tips for Exploration:\n');
fprintf('   - After `my_struct = some_bus_object.Values;`, type `my_struct` and press Enter to see fields.\n');
fprintf('   - Use `fieldnames(my_struct)` to get a cell array of field names.\n');
fprintf('   - Use Tab-completion in the Command Window: `my_struct.` then press Tab.\n');
fprintf('--- End of Parsing Instructions ---\n');

end % End of main function inspect_logsout_deep


% --- Helper Function (must be defined within the same file) ---
function inspect_signal_recursively(data_item, indent_str, max_depth_level)
% inspect_signal_recursively - Helper function to inspect bus structures.
% (This function remains unchanged from your provided version)

% --- Base Case for Recursion: Max depth reached or not a structure ---
if max_depth_level < 0 
    return; % Stop if max depth is exceeded for nested structures
end

if isstruct(data_item)
    fprintf('%sType: Bus (struct)\n', indent_str);
    
    busSignalNames = fieldnames(data_item);
    
    if isempty(busSignalNames)
        fprintf('%s  (This bus is empty)\n', indent_str);
        return;
    end
            
    fprintf('%s  Signals within this bus:\n', indent_str);
    for j = 1:length(busSignalNames)
        currentBusSignalName = busSignalNames{j};
        fprintf('%s    - %s\n', indent_str, currentBusSignalName);
        
        inspect_signal_recursively(data_item.(currentBusSignalName), [indent_str, '      '], max_depth_level - 1);
    end
else
    fprintf('%sType: Single Signal (Class: %s)\n', indent_str, class(data_item));
end

end % End of helper function inspect_signal_recursively
