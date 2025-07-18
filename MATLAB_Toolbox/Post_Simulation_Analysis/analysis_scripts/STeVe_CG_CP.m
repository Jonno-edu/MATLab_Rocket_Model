% Clear workspace, close existing figures, and clear command window
clear;
close all;
clc;

% --- Configuration ---
% Define the full paths for the data files.
% Note: The path for 'RocketSimData.mat' is assumed based on the other paths provided.
contour_file_path = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/input_data/STeVe Contour.xlsx';
simulation_data_path = '/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/Simulator_Core/input_data/RocketSimData.mat';

% --- Data Loading ---
% Load the necessary data
% Import rocket contour data from the specified Excel file path[8]
opts = spreadsheetImportOptions("NumVariables", 3);
opts.Sheet = "Sheet1";
opts.DataRange = "A2:C50";
opts.VariableNames = ["X", "Y", "Var3"];
opts.VariableTypes = ["double", "double", "string"];
opts = setvaropts(opts, "Var3", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Var3", "EmptyFieldRule", "auto");
STeVe_Contour = readtable(contour_file_path, opts, "UseExcel", false);
clear opts;

% Load mass and aerodynamic data from the specified .mat file path
data = load(simulation_data_path);
PrelookupData = data.PrelookupData;
AeroData = data.AeroData;

% --- Rocket Properties ---
% Get rocket dimensions
rocketLength = max(STeVe_Contour.X) - min(STeVe_Contour.X); % Total length in meters
rocketDiameter = data.RocketAero.Physical.Diameter; % Diameter in meters

% --- Figure and Plotting Setup ---
% Create the main figure window
fig = figure('Name', 'Rocket CG and CP Visualization', 'Position', [100, 100, 1200, 700]);

% Axis for the main rocket plot
ax = axes('Position', [0.08, 0.3, 0.6, 0.6]);

% Get rocket contour data for plotting
X = STeVe_Contour.X;
Y = STeVe_Contour.Y;

% Create a complete rocket outline by mirroring the Y-values
Y_complete = [-Y; flipud(Y)];
X_complete = [X; flipud(X)];

% Plot the rocket's outline
hold(ax, 'on');
rocket_outline = plot(ax, X_complete, Y_complete, 'k-', 'LineWidth', 2);

% Initialize markers for CG and CP
cg_marker = plot(ax, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
cp_marker = plot(ax, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Add a legend to the main plot
legend([rocket_outline, cg_marker, cp_marker], {'Rocket Outline', 'CG (from nose)', 'CP (from nose)'}, 'Location', 'best');

% Set axis properties for the main plot
title(ax, 'Rocket Stability Analysis');
xlabel(ax, 'Longitudinal Position (m)');
ylabel(ax, 'Radial Position (m)');
axis(ax, 'equal');
grid(ax, 'on');

% --- CP vs. Mach Plot ---
% Create axes for the CP vs. Mach number plot
cp_mach_ax = axes('Position', [0.75, 0.3, 0.2, 0.6]);

% Extract CP data for all Mach numbers at an angle of attack (alpha) of 0
alpha_idx = find(AeroData.Breakpoints.Alpha == 0, 1);
mach_values = AeroData.Breakpoints.Mach;
cp_values = zeros(size(mach_values));

for i = 1:length(mach_values)
    cp_values(i) = AeroData.Tables.CP(alpha_idx, i);
end

% Plot CP vs. Mach
plot(cp_mach_ax, mach_values, cp_values, 'b-', 'LineWidth', 2);
hold(cp_mach_ax, 'on');
cp_current_marker = plot(cp_mach_ax, mach_values(1), cp_values(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Set properties for the CP vs. Mach plot
title(cp_mach_ax, 'CP vs Mach Number');
xlabel(cp_mach_ax, 'Mach Number');
ylabel(cp_mach_ax, 'CP Position (m from nose)');
grid(cp_mach_ax, 'on');

% --- Data for Heatmaps ---
% Get all CG values over the simulation time
time_values = linspace(0, length(PrelookupData.Tables.COM_X)*PrelookupData.TimeStep, length(PrelookupData.Tables.COM_X));
cg_all = zeros(length(PrelookupData.Tables.COM_X), 1);
for i = 1:length(PrelookupData.Tables.COM_X)
    cg_all(i) = rocketLength - PrelookupData.Tables.COM_X(i);
end

% Create a visual representation (point cloud) of all possible CP positions
cp_colors = jet(length(cp_values));
scatter(ax, cp_values, zeros(size(cp_values)), 20, cp_colors, 'filled', 'o', 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.3);

% Create a visual representation (point cloud) of all possible CG positions
cg_colors = parula(length(cg_all));
scatter(ax, cg_all, zeros(size(cg_all))+0.01, 20, cg_colors, 'filled', 'o', 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.3);

% --- UI Controls ---
% Create sliders for interactivity
mach_slider = uicontrol('Style', 'slider', 'Position', [150, 30, 300, 20],...
    'Min', min(mach_values), 'Max', max(mach_values), 'Value', mach_values(1));
time_slider = uicontrol('Style', 'slider', 'Position', [600, 30, 300, 20],...
    'Min', min(time_values), 'Max', max(time_values), 'Value', time_values(1));

% Create labels for sliders
uicontrol('Style', 'text', 'Position', [50, 30, 100, 20], 'String', 'Mach Number:');
uicontrol('Style', 'text', 'Position', [500, 30, 100, 20], 'String', 'Time (s):');
mach_value = uicontrol('Style', 'text', 'Position', [460, 30, 100, 20], 'String', num2str(mach_values(1)));
time_value = uicontrol('Style', 'text', 'Position', [910, 30, 100, 20], 'String', num2str(time_values(1)));

% --- Trajectory Plot ---
% Add a plot for CG position over time
trajectory_ax = axes('Position', [0.08, 0.05, 0.87, 0.15]);
hold(trajectory_ax, 'on');
plot(trajectory_ax, time_values, cg_all, 'r-', 'LineWidth', 1.5);
ylabel(trajectory_ax, 'Position (m)');
xlabel(trajectory_ax, 'Time (s)');
title(trajectory_ax, 'CG Position vs Time');
grid(trajectory_ax, 'on');

% --- Callbacks and Initial Plot ---
% Set the functions to be called when sliders are moved
set(mach_slider, 'Callback', @updatePlot);
set(time_slider, 'Callback', @updatePlot);

% Call the update function once to initialize the plots
updatePlot();

% --- Nested Update Function ---
% This function is called whenever a slider value changes
function updatePlot(~,~)
    current_mach = get(mach_slider, 'Value');
    current_time = get(time_slider, 'Value');
    
    % Update the text displays for current slider values
    set(mach_value, 'String', sprintf('%.2f', current_mach));
    set(time_value, 'String', sprintf('%.2f', current_time));
    
    % Find the closest data indices for the current slider values
    [~, mach_idx] = min(abs(mach_values - current_mach));
    time_idx = round(current_time/PrelookupData.TimeStep) + 1;
    time_idx = max(1, min(time_idx, length(PrelookupData.Tables.COM_X)));
    
    % Get CP position for the current Mach number
    cp_position = AeroData.Tables.CP(alpha_idx, mach_idx);
    
    % Get CG position for the current time
    cg_position = rocketLength - PrelookupData.Tables.COM_X(time_idx);
    
    % Update the markers on the main rocket plot
    set(cp_marker, 'XData', cp_position, 'YData', 0);
    set(cg_marker, 'XData', cg_position, 'YData', 0);
    
    % Update the marker on the CP vs. Mach plot
    set(cp_current_marker, 'XData', current_mach, 'YData', cp_position);
    
    % Calculate the stability margin
    stability_margin_m = cp_position - cg_position;
    stability_calibers = stability_margin_m / rocketDiameter;
    
    % Update the main plot's title with stability information
    title_str = {
        sprintf('CP: %.2f m from nose', cp_position)
        sprintf('CG: %.2f m from nose', cg_position)
        sprintf('Stability Margin: %.2f m (%.1f calibers)', stability_margin_m, stability_calibers)     
    };
    title(ax, title_str);
    
    % Add a stability status indicator to the main plot
    if stability_calibers >= 1
        status = {'STABLE', '(Margin >= 1 caliber)'};
        color = [0 0.7 0]; % Green
    elseif stability_calibers > 0
        status = {'MARGINALLY STABLE', '(Positive but <1 caliber)'};
        color = [1 0.5 0]; % Orange
    else
        status = {'UNSTABLE', '(Negative margin)'};
        color = [0.8 0 0]; % Red
    end
    
    % Update the status text object, or create it if it doesn't exist
    status_text_handle = findobj(ax, 'Tag', 'StatusText');
    if isempty(status_text_handle)
        text(0.1, 0.9, status, 'Units', 'normalized', 'Color', color,...
            'FontWeight', 'bold', 'FontSize', 12, 'Parent', ax, 'Tag', 'StatusText');
    else
        set(status_text_handle, 'String', status, 'Color', color);
    end
    
    % Mark the current points in the point clouds for better visibility
    current_cp_handle = findobj(ax, 'Tag', 'CurrentCPPoint');
    if isempty(current_cp_handle)
        scatter(ax, cp_position, 0, 100, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'Tag', 'CurrentCPPoint');
    else
        set(current_cp_handle, 'XData', cp_position);
    end
    
    current_cg_handle = findobj(ax, 'Tag', 'CurrentCGPoint');
    if isempty(current_cg_handle)
        scatter(ax, cg_position, 0.01, 100, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'Tag', 'CurrentCGPoint');
    else
        set(current_cg_handle, 'XData', cg_position);
    end
    
    % Update the trajectory plot with a marker for the current time
    time_marker_handle = findobj(trajectory_ax, 'Tag', 'CurrentTimeMarker');
    if isempty(time_marker_handle)
        plot(trajectory_ax, [current_time current_time], ylim(trajectory_ax), 'k--', 'LineWidth', 1.5, 'Tag', 'CurrentTimeMarker');
    else
        set(time_marker_handle, 'XData', [current_time current_time]);
    end
end
