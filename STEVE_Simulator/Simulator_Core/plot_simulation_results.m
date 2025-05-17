function plot_simulation_results(simOut)
% plot_simulation_results - Plots key flight data from the simulation output in tabs.
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')

if ~exist('simOut', 'var') || isempty(simOut)
    disp('simOut variable not provided or is empty.');
    return;
end

try
    if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
        logsout = simOut.logsout;
    else
        disp('logsout not found in simOut or is empty. Please check signal logging.');
        return;
    end

    % Create a figure with a tab group
    hFig = figure('Name', 'Rocket Flight Performance Data', 'NumberTitle', 'off', 'WindowState', 'maximized');
    hTabGroup = uitabgroup(hFig);

    % --- Access PlantData Structure ---
    try
        plant_signal_object = logsout.getElement('PlantData');
        plant_data_structure = plant_signal_object.Values; % This is the struct
    catch ME_plantdata
        disp('Error accessing PlantData from logsout. Ensure "PlantData" is logged as a bus.');
        disp(ME_plantdata.message);
        return; % Can't proceed without PlantData for most plots
    end

    % --- Tab 1: Trajectory and Position (2x2 subplots) ---
    hTabTrajectory = uitab(hTabGroup, 'Title', 'Trajectory & Position');
    try
        % Extract Xe (Earth-fixed position [X, Y, Z])
        if isfield(plant_data_structure, 'Xe')
            Xe_timeseries = plant_data_structure.Xe;
            time_Xe = Xe_timeseries.Time;
            X_pos = Xe_timeseries.Data(:,1); % Assuming X is 1st column
            Y_pos = Xe_timeseries.Data(:,2); % Assuming Y is 2nd column
            % MODIFIED: Z_pos is now positive upwards (Altitude)
            Z_pos_up = -Xe_timeseries.Data(:,3); 

            % Subplot 1: 3D Trajectory
            ax3D = subplot(2,2,1, 'Parent', hTabTrajectory);
            plot3(ax3D, X_pos, Y_pos, Z_pos_up);
            xlabel(ax3D, 'X_e (m)'); ylabel(ax3D, 'Y_e (m)'); zlabel(ax3D, 'Altitude (Z_e up, m)');
            title(ax3D, '3D Trajectory');
            grid(ax3D, 'on'); 
            % MODIFIED: Removed axis equal to allow independent scaling
            axis(ax3D, 'normal'); % Explicitly set to default auto-scaling
            view(ax3D, 3);

            % Subplot 2: Groundtrack (X vs Y)
            axGT = subplot(2,2,2, 'Parent', hTabTrajectory);
            plot(axGT, X_pos, Y_pos);
            xlabel(axGT, 'X_e (m)'); ylabel(axGT, 'Y_e (m)');
            title(axGT, 'Groundtrack');
            grid(axGT, 'on'); axis(axGT, 'equal'); % Keep groundtrack equal scaled

            % Subplot 3: Height (Altitude) over Time
            axHeight = subplot(2,2,3, 'Parent', hTabTrajectory);
            plot(axHeight, time_Xe, Z_pos_up);
            xlabel(axHeight, 'Time (s)'); ylabel(axHeight, 'Altitude (Z_e up, m)');
            title(axHeight, 'Altitude over Time');
            grid(axHeight, 'on');

            % Subplot 4: Ground Distance over Time
            axDist = subplot(2,2,4, 'Parent', hTabTrajectory);
            ground_dist = zeros(size(X_pos));
            if length(X_pos) > 1 % Ensure there's more than one point to calculate diff
                 ground_dist(2:end) = cumsum(sqrt(diff(X_pos).^2 + diff(Y_pos).^2));
            end
            plot(axDist, time_Xe, ground_dist);
            xlabel(axDist, 'Time (s)'); ylabel(axDist, 'Ground Distance (m)');
            title(axDist, 'Ground Distance over Time');
            grid(axDist, 'on');
        else
            disp('Position data (PlantData.Xe) not found.');
            text(0.5, 0.5, 'Position data (PlantData.Xe) not found.', 'Parent', hTabTrajectory, 'HorizontalAlignment', 'center');
        end
    catch ME_traj
        disp('Error plotting Trajectory & Position tab:');
        disp(getReport(ME_traj, 'extended', 'hyperlinks','on'));
        text(0.5, 0.5, 'Error. Check Command Window.', 'Parent', hTabTrajectory, 'HorizontalAlignment', 'center');
    end


    % --- Tab 2: Velocities ---
    hTabVelocities = uitab(hTabGroup, 'Title', 'Velocities');
    try
        if isfield(plant_data_structure, 'Ve')
            Ve_timeseries = plant_data_structure.Ve;
            time_Ve = Ve_timeseries.Time;
            Vx_earth = Ve_timeseries.Data(:,1); % Assuming Vx is 1st column
            Vy_earth = Ve_timeseries.Data(:,2); % Assuming Vy is 2nd column
            % MODIFIED: Vz_up is now positive upwards
            Vz_up = -Ve_timeseries.Data(:,3); 

            % Subplot 1: Horizontal Velocities
            axVelHoriz = subplot(2,1,1, 'Parent', hTabVelocities);
            plot(axVelHoriz, time_Ve, Vx_earth, 'DisplayName', 'V_x (Earth Frame)');
            hold(axVelHoriz, 'on');
            plot(axVelHoriz, time_Ve, Vy_earth, 'DisplayName', 'V_y (Earth Frame)');
            hold(axVelHoriz, 'off');
            ylabel(axVelHoriz, 'Horizontal Velocity (m/s)');
            title(axVelHoriz, 'Horizontal Velocities Over Time');
            grid(axVelHoriz, 'on');
            legend(axVelHoriz, 'show', 'Location', 'best');
            
            % Subplot 2: Vertical Velocity
            axVelVert = subplot(2,1,2, 'Parent', hTabVelocities);
            plot(axVelVert, time_Ve, Vz_up, 'DisplayName', 'V_z (Up is positive)');
            xlabel(axVelVert, 'Time (s)');
            ylabel(axVelVert, 'Vertical Velocity (m/s)');
            title(axVelVert, 'Vertical Velocity Over Time');
            grid(axVelVert, 'on');
            legend(axVelVert, 'show', 'Location', 'best');
        else
            disp('Velocity data (PlantData.Ve) not found.');
             % Add text to both potential subplots if data is missing
            axVelHoriz = subplot(2,1,1, 'Parent', hTabVelocities);
            text(0.5, 0.5, 'Velocity data (PlantData.Ve) not found.', 'Parent', axVelHoriz, 'HorizontalAlignment', 'center');
            axVelVert = subplot(2,1,2, 'Parent', hTabVelocities);
            text(0.5, 0.5, 'Velocity data (PlantData.Ve) not found.', 'Parent', axVelVert, 'HorizontalAlignment', 'center');
        end
    catch ME_vel
        disp('Error plotting Velocities tab:');
        disp(getReport(ME_vel, 'extended', 'hyperlinks','on'));
        % Add text to both potential subplots on error
        axVelHoriz = subplot(2,1,1, 'Parent', hTabVelocities);
        text(0.5, 0.5, 'Error. Check Command Window.', 'Parent', axVelHoriz, 'HorizontalAlignment', 'center');
        axVelVert = subplot(2,1,2, 'Parent', hTabVelocities);
        text(0.5, 0.5, 'Error. Check Command Window.', 'Parent', axVelVert, 'HorizontalAlignment', 'center');
    end

    % --- Tab 3: Attitude & Control ---
    hTabAttitude = uitab(hTabGroup, 'Title', 'Attitude & Control');
    try
        % Subplot 1: Pitch Angle
        axPitch = subplot(4,1,1, 'Parent', hTabAttitude);
        if isfield(plant_data_structure, 'body_angles') && isstruct(plant_data_structure.body_angles) && isfield(plant_data_structure.body_angles, 'theta')
            pitch_angle_timeseries = plant_data_structure.body_angles.theta;
            plot(axPitch, pitch_angle_timeseries.Time, rad2deg(pitch_angle_timeseries.Data));
            ylabel(axPitch, 'Pitch (\theta, deg)'); title(axPitch, 'Pitch Angle'); grid(axPitch, 'on');
        else
            text(0.5,0.5,'Pitch (PlantData.body_angles.theta) not found','Parent',axPitch,'HorizontalAlignment','center');
        end

        % Subplot 2: Pitch Rate
        axPitchRate = subplot(4,1,2, 'Parent', hTabAttitude);
        if isfield(plant_data_structure, 'w')
            angular_rates_timeseries = plant_data_structure.w;
            if size(angular_rates_timeseries.Data, 2) >= 2
                plot(axPitchRate, angular_rates_timeseries.Time, rad2deg(angular_rates_timeseries.Data(:,2)));
                ylabel(axPitchRate, 'Pitch Rate (q, deg/s)'); title(axPitchRate, 'Pitch Rate'); grid(axPitchRate, 'on');
            else
                text(0.5,0.5,'Pitch Rate (PlantData.w(:,2)) format error','Parent',axPitchRate,'HorizontalAlignment','center');
            end
        else
            text(0.5,0.5,'Pitch Rate (PlantData.w) not found','Parent',axPitchRate,'HorizontalAlignment','center');
        end

        % Subplot 3: Alpha (Angle of Attack) - MODIFIED
        axAlpha = subplot(4,1,3, 'Parent', hTabAttitude);
        if isfield(plant_data_structure, 'alpha')
            alpha_timeseries = plant_data_structure.alpha;
            alpha_data_modified = alpha_timeseries.Data; % Get a copy of the data
            
            % Check if there are at least 2 data points to modify
            if length(alpha_data_modified) >= 2
                alpha_data_modified(1:2) = 0; % Set first two data points to 0
            elseif ~isempty(alpha_data_modified) % If only one point, set it to 0
                alpha_data_modified(1) = 0;
            end
            
            plot(axAlpha, alpha_timeseries.Time, rad2deg(alpha_data_modified));
            ylabel(axAlpha, 'Alpha (\alpha, deg)'); title(axAlpha, 'Angle of Attack (Initial points zeroed)'); grid(axAlpha, 'on');
        else
            text(0.5,0.5,'Alpha (PlantData.alpha) not found','Parent',axAlpha,'HorizontalAlignment','center');
        end

        % Subplot 4: Nozzle Angle
        axNozzle = subplot(4,1,4, 'Parent', hTabAttitude);
        if any(strcmp(logsout.getElementNames(), 'Y_nozzle_angle'))
            nozzle_signal_object = logsout.getElement('Y_nozzle_angle');
            nozzle_angle_timeseries = nozzle_signal_object.Values;
            plot(axNozzle, nozzle_angle_timeseries.Time, rad2deg(nozzle_angle_timeseries.Data));
            ylabel(axNozzle, 'Nozzle Angle (\delta_y, deg)'); title(axNozzle, 'Nozzle Angle'); grid(axNozzle, 'on');
        else
            text(0.5,0.5,'Nozzle Angle (Y_nozzle_angle) not found','Parent',axNozzle,'HorizontalAlignment','center');
        end
        xlabel(axNozzle, 'Time (s)'); % Common X-label for this tab

    catch ME_att
        disp('Error plotting Attitude & Control tab:');
        disp(getReport(ME_att, 'extended', 'hyperlinks','on'));
        text(0.5, 0.5, 'Error. Check Command Window.', 'Parent', hTabAttitude, 'HorizontalAlignment', 'center');
    end

catch ME_main
    disp('An error occurred in plot_simulation_results:');
    disp(ME_main.message);
    disp(ME_main.getReport('extended', 'hyperlinks','on'));
end

end
