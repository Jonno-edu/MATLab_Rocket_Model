function plot_simulation_results(simOut)
% plot_simulation_results - Plots key flight data from the simulation output in tabs.
% Inputs:
%   simOut: The output struct from the Simulink simulation (e.g., 'out')

if ~exist('simOut', 'var') || isempty(simOut)
    disp('plot_simulation_results: simOut variable not provided or is empty.');
    return;
end

try % Main try block for the whole function
    if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
        logsout = simOut.logsout;
    else
        disp('plot_simulation_results: logsout not found in simOut or is empty. Please check signal logging.');
        return;
    end

    hFig = figure('Name', 'Rocket Flight Performance Data', 'NumberTitle', 'off', 'WindowState', 'maximized');
    hTabGroup = uitabgroup(hFig);

    allLoggedSignalNames = logsout.getElementNames(); 

    pd = []; 
    try
        if any(strcmp(allLoggedSignalNames, 'PlantData')) 
            plant_signal_object = logsout.getElement('PlantData');
            pd = plant_signal_object.Values; 
        else
            disp('Warning: "PlantData" bus not found in logsout. Some plots may be empty or skipped.');
        end
    catch ME_plantdata
        disp('Error accessing PlantData from logsout:'); disp(ME_plantdata.message);
    end 
    
    cmd_data_structure = []; 
    try
        if any(strcmp(allLoggedSignalNames, 'CMD')) 
            cmd_signal_object = logsout.getElement('CMD');
            cmd_data_structure = cmd_signal_object.Values; 
            if isempty(cmd_data_structure)
                disp('INFO: CMD bus was found but is empty.');
            elseif ~isstruct(cmd_data_structure)
                disp('WARNING: CMD bus signal was found but .Values is not a struct.');
                cmd_data_structure = []; 
            else
                disp('INFO: CMD bus accessed successfully.');
            end
        else
            disp('INFO: CMD bus not found in logsout. Command plots will be skipped.');
        end
    catch ME_cmd_access
        disp('Error accessing CMD bus from logsout:'); disp(ME_cmd_access.message);
    end 

    % --- Tab 1: Attitude & Control ---
    hTabAttitude = uitab(hTabGroup, 'Title', 'Attitude & Control');
    try
        axPitch = subplot(4,1,1, 'Parent', hTabAttitude); hold(axPitch, 'on'); grid(axPitch, 'on');
        plotCountPitch = 0; 
        actualThetaTime = []; 

        if ~isempty(pd) && isfield(pd, 'body_angles') && isstruct(pd.body_angles) && ...
           isfield(pd.body_angles, 'theta') && isa(pd.body_angles.theta, 'timeseries') && ...
           ~isempty(pd.body_angles.theta.Time)
            
            actualTheta_ts = pd.body_angles.theta;
            actualThetaTime = actualTheta_ts.Time; 
            plot(axPitch, actualThetaTime, rad2deg(squeeze(double(actualTheta_ts.Data))), 'b-', 'DisplayName', '\theta (Actual)');
            plotCountPitch = plotCountPitch + 1;
        else
            if plotCountPitch == 0 
                text(0.5,0.5,'\theta (Actual) missing or invalid.','Parent',axPitch,'HorizontalAlignment','center','Interpreter','tex');
            end
        end
        
        disp('Attempting to plot ThetaCmd...');
        if ~isempty(cmd_data_structure) && isstruct(cmd_data_structure) && isfield(cmd_data_structure, 'ThetaCmd') && ...
           isa(cmd_data_structure.ThetaCmd, 'timeseries') && ~isempty(cmd_data_structure.ThetaCmd.Time)
            
            thetaCmd_ts = cmd_data_structure.ThetaCmd;
            thetaCmd_values_deg = rad2deg(squeeze(double(thetaCmd_ts.Data)));
            
            if length(thetaCmd_ts.Time) == 1 
                fprintf('Plotting ThetaCmd as a horizontal line (single point command). Value: %.2f deg\n', thetaCmd_values_deg(1)); 
                yline(axPitch, thetaCmd_values_deg(1), 'r--', ... 
                    'DisplayName', '\theta_{cmd} (Constant)', ...
                    'LineWidth', 1.5, 'Tag', 'ThetaCmdLine'); 
                plotCountPitch = plotCountPitch + 1;
            elseif length(thetaCmd_ts.Time) > 1 
                fprintf('Plotting ThetaCmd as a trajectory. Number of time points: %d\n', length(thetaCmd_ts.Time));
                plot(axPitch, thetaCmd_ts.Time, thetaCmd_values_deg, ...
                     'r--o', 'MarkerSize', 4, 'LineWidth', 1.5, 'DisplayName', '\theta_{cmd}', 'Tag', 'ThetaCmdLine');
                plotCountPitch = plotCountPitch + 1;
            else
                disp('INFO: ThetaCmd has no time points, not plotted.');
            end
        else
            disp('INFO: ThetaCmd not plotted. Reasons:');
            if isempty(cmd_data_structure), disp('  - CMD data structure is empty.');
            elseif ~isstruct(cmd_data_structure), disp('  - CMD data is not a struct.');
            elseif ~isfield(cmd_data_structure, 'ThetaCmd'), disp('  - ThetaCmd field missing in CMD bus.');
            elseif isfield(cmd_data_structure, 'ThetaCmd') && ~isa(cmd_data_structure.ThetaCmd, 'timeseries'), disp('  - ThetaCmd is not a timeseries object.');
            elseif isfield(cmd_data_structure, 'ThetaCmd') && isa(cmd_data_structure.ThetaCmd, 'timeseries') && isempty(cmd_data_structure.ThetaCmd.Time), disp('  - ThetaCmd timeseries has no time points.');
            else disp('  - Other unspecified reason for ThetaCmd.'); end
        end
        ylabel(axPitch, '\theta (deg)', 'Interpreter', 'tex'); title(axPitch, 'Pitch Angle & Command'); 
        
        linesWithDisplayName = findobj(axPitch, 'Type', 'line', '-and', '-not', 'DisplayName', '');
        constantLinesWithDisplayName = findobj(axPitch, 'Type', 'ConstantLine', '-and', '-not', 'DisplayName', '');
        allPlottedObjects = [linesWithDisplayName; constantLinesWithDisplayName];
        
        if ~isempty(allPlottedObjects)
            legend(axPitch, allPlottedObjects, 'Location','best','Interpreter','tex');
        end
        hold(axPitch, 'off');

        axAlpha = subplot(4,1,2, 'Parent', hTabAttitude);
        if ~isempty(pd) && isfield(pd, 'alpha') && isa(pd.alpha, 'timeseries')
            alpha_ts = pd.alpha; alpha_data_mod = squeeze(double(alpha_ts.Data)); 
            if length(alpha_data_mod) >= 2, alpha_data_mod(1:2) = 0; elseif ~isempty(alpha_data_mod), alpha_data_mod(1) = 0; end
            plot(axAlpha, alpha_ts.Time, rad2deg(alpha_data_mod));
            ylabel(axAlpha, '\alpha (deg)', 'Interpreter', 'tex'); title(axAlpha, 'Angle of Attack'); grid(axAlpha, 'on');
        else text(0.5,0.5,'\alpha missing','Parent',axAlpha,'HorizontalAlignment','center','Interpreter','tex'); end

        axPitchRate = subplot(4,1,3, 'Parent', hTabAttitude);
        if ~isempty(pd) && isfield(pd, 'w') && isa(pd.w, 'timeseries')
            w_data = squeeze(double(pd.w.Data));
            if size(w_data, 2) >= 2, plot(axPitchRate, pd.w.Time, rad2deg(w_data(:,2))); else text(0.5,0.5,'w data <2 col','Parent',axPitchRate,'HorizontalAlignment','center'); end
            ylabel(axPitchRate, 'q (deg/s)'); title(axPitchRate, 'Pitch Rate'); grid(axPitchRate, 'on');
        else text(0.5,0.5,'w missing','Parent',axPitchRate,'HorizontalAlignment','center'); end
        
        axNozzle = subplot(4,1,4, 'Parent', hTabAttitude);
        found_nozzle = false; nozzle_data_to_plot = []; nozzle_time_to_plot = [];
        if ~isempty(pd) && isfield(pd, 'Actuators') && isstruct(pd.Actuators) && isfield(pd.Actuators, 'Y_nozzle_angle') && isa(pd.Actuators.Y_nozzle_angle, 'timeseries')
            nozzle_data_to_plot = pd.Actuators.Y_nozzle_angle.Data; nozzle_time_to_plot = pd.Actuators.Y_nozzle_angle.Time; found_nozzle = true;
        elseif any(strcmp(allLoggedSignalNames, 'Y_nozzle_angle')) 
            nozzle_obj_top_el = logsout.getElement('Y_nozzle_angle'); 
            if isa(nozzle_obj_top_el.Values, 'timeseries')
                nozzle_data_to_plot = nozzle_obj_top_el.Values.Data; 
                nozzle_time_to_plot = nozzle_obj_top_el.Values.Time; 
                found_nozzle = true; 
            end
        end
        if found_nozzle && ~isempty(nozzle_time_to_plot) && ~isempty(nozzle_data_to_plot)
            plot(axNozzle, nozzle_time_to_plot, rad2deg(squeeze(double(nozzle_data_to_plot)))); 
            ylabel(axNozzle, '\delta_y (deg)', 'Interpreter', 'tex'); title(axNozzle, 'Nozzle Angle'); grid(axNozzle, 'on');
        else 
            text(0.5,0.5,'\delta_y missing','Parent',axNozzle,'HorizontalAlignment','center','Interpreter','tex'); 
        end
        xlabel(axNozzle, 'Time (s)');
    catch ME_att
        disp('Error plotting Attitude Tab:'); disp(getReport(ME_att)); 
    end 

    % --- Tab 2: Trajectory and Position ---
    hTabTrajectory = uitab(hTabGroup, 'Title', 'Trajectory & Position');
    try
        if ~isempty(pd) && isfield(pd, 'Xe') && isa(pd.Xe, 'timeseries') && size(pd.Xe.Data,2) >=3
            Xe_ts = pd.Xe; time_Xe = Xe_ts.Time; X_pos = Xe_ts.Data(:,1); Y_pos = Xe_ts.Data(:,2); Z_pos_up = -Xe_ts.Data(:,3); 
            subplot(2,2,1, 'Parent', hTabTrajectory); plot3(X_pos, Y_pos, Z_pos_up); xlabel('X_e (m)'); ylabel('Y_e (m)'); zlabel('Alt (Z_e up, m)'); title('3D Trajectory'); grid on; axis normal; view(3);
            subplot(2,2,2, 'Parent', hTabTrajectory); plot(X_pos, Y_pos); xlabel('X_e (m)'); ylabel('Y_e (m)'); title('Groundtrack'); grid on; axis equal;
            subplot(2,2,3, 'Parent', hTabTrajectory); plot(time_Xe, Z_pos_up); xlabel('Time (s)'); ylabel('Alt (Z_e up, m)'); title('Altitude'); grid on;
            axDist=subplot(2,2,4,'Parent',hTabTrajectory); ground_dist=zeros(size(X_pos)); if length(X_pos)>1, ground_dist(2:end)=cumsum(sqrt(diff(X_pos).^2+diff(Y_pos).^2)); end; plot(axDist,time_Xe,ground_dist); xlabel('Time (s)'); ylabel('Ground Dist (m)'); title('Ground Distance'); grid on;
        else
            text(0.5,0.5,'Position data (PlantData.Xe) missing/invalid.','Parent',hTabTrajectory,'HorizontalAlignment','center');
        end
    catch ME_traj
        disp('Error plotting Trajectory Tab:'); disp(getReport(ME_traj)); 
    end 

    % --- Tab 3: Velocities & Accelerations --- (Expanded to 2x2)
    hTabVelocities = uitab(hTabGroup, 'Title', 'Velocities & Accels'); 
    try
        % Velocity Plots
        if ~isempty(pd) && isfield(pd, 'Ve') && isa(pd.Ve, 'timeseries') && size(pd.Ve.Data,2) >=3
            Ve_ts = pd.Ve; time_Ve = Ve_ts.Time; Vx_e = Ve_ts.Data(:,1); Vy_e = Ve_ts.Data(:,2); Vz_up = -Ve_ts.Data(:,3);
            
            axVH = subplot(2,2,1,'Parent',hTabVelocities); 
            plot(axVH,time_Ve,Vx_e,'DisplayName','V_x (Earth)'); hold(axVH,'on'); 
            plot(axVH,time_Ve,Vy_e,'DisplayName','V_y (Earth)'); hold(axVH,'off'); 
            ylabel(axVH,'Horiz. Vel (m/s)'); title(axVH,'Horizontal Velocities (Earth Frame)'); 
            grid(axVH,'on'); legend(axVH,'show','Location','best');
            
            axVV = subplot(2,2,2,'Parent',hTabVelocities); 
            plot(axVV,time_Ve,Vz_up,'DisplayName','V_z (Up)'); 
            ylabel(axVV,'Vert. Vel (m/s)'); title(axVV,'Vertical Velocity (Earth Frame, Up is +)'); 
            grid(axVV,'on'); legend(axVV,'show','Location','best');
        else
            subplot(2,2,1,'Parent',hTabVelocities); text(0.5,0.5,'Earth Vel data (PlantData.Ve) missing.','HA','center'); title('Horizontal Velocities (Earth Frame)');
            subplot(2,2,2,'Parent',hTabVelocities); text(0.5,0.5,'Earth Vel data (PlantData.Ve) missing.','HA','center'); title('Vertical Velocity (Earth Frame, Up is +)');
        end

        % Acceleration Plots
        if ~isempty(pd) && isfield(pd, 'Ab') && isa(pd.Ab, 'timeseries') && size(pd.Ab.Data,2) >=3
            Ab_ts = pd.Ab; time_Ab = Ab_ts.Time; 
            Ax_b = Ab_ts.Data(:,1); Ay_b = Ab_ts.Data(:,2); Az_b = Ab_ts.Data(:,3);

            axAH = subplot(2,2,3,'Parent',hTabVelocities);
            plot(axAH,time_Ab,Ax_b,'DisplayName','Ax (Body)'); hold(axAH,'on');
            plot(axAH,time_Ab,Ay_b,'DisplayName','Ay (Body)'); hold(axAH,'off');
            xlabel(axAH,'Time (s)'); ylabel(axAH,'Horiz. Accel (m/s^2)'); 
            title(axAH,'Body X & Y Accelerations'); 
            grid(axAH,'on'); legend(axAH,'show','Location','best');

            axAV = subplot(2,2,4,'Parent',hTabVelocities);
            plot(axAV,time_Ab,Az_b,'DisplayName','Az (Body)');
            xlabel(axAV,'Time (s)'); ylabel(axAV,'Vert. Accel (m/s^2)'); 
            title(axAV,'Body Z Acceleration'); 
            grid(axAV,'on'); legend(axAV,'show','Location','best');
        else
             subplot(2,2,3,'Parent',hTabVelocities); text(0.5,0.5,'Body Accel data (PlantData.Ab) missing.','HA','center'); title('Body X & Y Accelerations');  xlabel('Time (s)');
             subplot(2,2,4,'Parent',hTabVelocities); text(0.5,0.5,'Body Accel data (PlantData.Ab) missing.','HA','center'); title('Body Z Acceleration'); xlabel('Time (s)');
        end
    catch ME_vel_accel
        disp('Error plotting Velocities & Accelerations Tab:'); disp(getReport(ME_vel_accel)); 
    end 
    
    % --- Tab 4: Environment & Flight Conditions (Consolidated) ---
    hTabEnvFlight = uitab(hTabGroup, 'Title', 'Environment & Flight'); 
    try
        envBusSignalDefs = { 
            {'WindVelocity', 'Wind Velocity', 'm/s', {'Vx_{wind}', 'Vy_{wind}', 'Vz_{wind}'}}, 
            {'To', 'Ambient Temperature', 'K', {'Temperature'}},                               
            {'a', 'Speed of Sound', 'm/s', {'Speed of Sound'}},                                 
            {'Po', 'Ambient Pressure', 'Pa', {'Pressure'}},                                     
            {'AirDensity', 'Air Density', 'kg/m^3', {'Density'}}                                     
        };
        topLevelFlightSignalDefs = {
            {'mach', 'Mach Number', '', {'Mach'}},         
            {'qbar', 'Dynamic Pressure (qbar)', 'Pa', {'qbar'}},    
            {'airspeed', 'Airspeed', 'm/s', {'Airspeed'}}       
        };
        
        numValidEnvBusSignals = 0;
        if ~isempty(envBusSignalDefs) && iscell(envBusSignalDefs)
            for i = 1:size(envBusSignalDefs,1)
                if iscell(envBusSignalDefs{i}) && numel(envBusSignalDefs{i}) == 4 && iscell(envBusSignalDefs{i}{4})
                    numValidEnvBusSignals = numValidEnvBusSignals + 1;
                else fprintf('INFO Tab4: Malformed row %d in envBusSignalDefs, will be skipped.\n', i); disp(envBusSignalDefs{i}); end
            end
        end
        numValidTopLevelSignals = 0;
        if ~isempty(topLevelFlightSignalDefs) && iscell(topLevelFlightSignalDefs)
            for i = 1:size(topLevelFlightSignalDefs,1)
                if iscell(topLevelFlightSignalDefs{i}) && numel(topLevelFlightSignalDefs{i}) == 4 && iscell(topLevelFlightSignalDefs{i}{4})
                    numValidTopLevelSignals = numValidTopLevelSignals + 1;
                else fprintf('INFO Tab4: Malformed row %d in topLevelFlightSignalDefs, will be skipped.\n', i); disp(topLevelFlightSignalDefs{i}); end
            end
        end

        nTotalPlotsForTab4 = numValidEnvBusSignals + numValidTopLevelSignals;
        fprintf('Env & Flight Tab: numValidEnvBusSignals = %d, numValidTopLevelSignals = %d, nTotalPlotsForTab4 = %d\n', numValidEnvBusSignals, numValidTopLevelSignals, nTotalPlotsForTab4);

        nRowsTab4 = ceil(sqrt(nTotalPlotsForTab4)); if nTotalPlotsForTab4 == 0, nRowsTab4=1; end 
        nColsTab4 = ceil(nTotalPlotsForTab4/nRowsTab4); if nTotalPlotsForTab4 == 0, nColsTab4=1; end
        currentPlotIndex = 0; 
        
        envBus = []; 
        if any(strcmp(allLoggedSignalNames, 'ENV')), envBusCandidate = logsout.getElement('ENV').Values; if isa(envBusCandidate, 'struct'), envBus = envBusCandidate; else disp('Warning: ENV is not a struct.'); end
        else disp('Warning: ENV bus not found.'); end

        if ~isempty(envBusSignalDefs) && iscell(envBusSignalDefs)
            for k_env = 1:size(envBusSignalDefs,1)
                if ~(iscell(envBusSignalDefs{k_env}) && numel(envBusSignalDefs{k_env}) == 4 && iscell(envBusSignalDefs{k_env}{4})), continue; end
                origName=envBusSignalDefs{k_env}{1}; plotName=envBusSignalDefs{k_env}{2}; plotUnit=envBusSignalDefs{k_env}{3}; componentLegendNames=envBusSignalDefs{k_env}{4};
                currentPlotIndex = currentPlotIndex + 1; ax = subplot(nRowsTab4,nColsTab4,currentPlotIndex,'Parent',hTabEnvFlight); ts=[];
                
                if ~isempty(envBus) && isfield(envBus,origName) && isa(envBus.(origName),'timeseries'), ts=envBus.(origName);
                else fprintf('  Signal "%s" NOT found/valid in ENV bus for plotting (Tab4).\n', origName); end

                if ~isempty(ts)
                    dataToPlot=squeeze(double(ts.Data)); hold(ax,'on'); plotMade=false; legendEntries={};
                    if isvector(dataToPlot) && ~isempty(dataToPlot), currentDisplayName=componentLegendNames{1}; plot(ax,ts.Time,dataToPlot,'DisplayName',currentDisplayName); legendEntries={currentDisplayName}; plotMade=true;
                    elseif size(dataToPlot,2)>=1 && size(dataToPlot,2)<=3 && ~isempty(dataToPlot), for compIdx=1:size(dataToPlot,2),currentCompName=componentLegendNames{min(compIdx,numel(componentLegendNames))}; plot(ax,ts.Time,dataToPlot(:,compIdx),'DisplayName',currentCompName); legendEntries{end+1}=currentCompName; end; plotMade=true;
                    elseif size(dataToPlot,2)>3 && ~isempty(dataToPlot), currentDisplayName=[componentLegendNames{1} ' (Comp.1)']; plot(ax,ts.Time,dataToPlot(:,1),'DisplayName',currentDisplayName); legendEntries={currentDisplayName}; disp(['Signal ',origName,' >3 comp; plotting first.']); plotMade=true;
                    end
                    if plotMade && ~isempty(legendEntries), legend(ax,legendEntries,'Location','best','Interpreter','tex'); else text(0.5,0.5,[plotName,' (Data Empty/Unhandled)'],'Parent',ax,'HA','center'); end
                    hold(ax,'off'); title(ax,plotName); xlabel(ax,'Time (s)'); ylabel(ax,plotUnit); grid(ax,'on');
                else text(0.5,0.5,[plotName,' (Signal Data Not Available)'],'Parent',ax,'HA','center'); end
            end
        end
        if ~isempty(topLevelFlightSignalDefs) && iscell(topLevelFlightSignalDefs)
            for k_top = 1:size(topLevelFlightSignalDefs,1)
                 if ~(iscell(topLevelFlightSignalDefs{k_top}) && numel(topLevelFlightSignalDefs{k_top}) == 4 && iscell(topLevelFlightSignalDefs{k_top}{4})), continue; end
                origName=topLevelFlightSignalDefs{k_top}{1}; plotName=topLevelFlightSignalDefs{k_top}{2}; plotUnit=topLevelFlightSignalDefs{k_top}{3}; componentLegendNames=topLevelFlightSignalDefs{k_top}{4};
                currentPlotIndex = currentPlotIndex + 1; ax = subplot(nRowsTab4,nColsTab4,currentPlotIndex,'Parent',hTabEnvFlight); ts=[];

                if any(strcmp(allLoggedSignalNames,origName)), signalObj=logsout.getElement(origName); if isa(signalObj.Values,'timeseries'), ts=signalObj.Values; 
                else fprintf('  Signal "%s" (Top-Level) NOT a timeseries (Tab4).\n',origName); end
                else fprintf('  Signal "%s" NOT found at Top-Level (Tab4).\n',origName); end
                if ~isempty(ts)
                    dataToPlot = squeeze(double(ts.Data)); 
                    if isvector(dataToPlot) && ~isempty(dataToPlot), plot(ax,ts.Time,dataToPlot,'DisplayName',componentLegendNames{1}); legend(ax,'show','Location','best','Interpreter','tex'); 
                    else text(0.5,0.5,[plotName,' (Data Invalid/Empty)'],'Parent',ax,'HA','center'); end
                    title(ax,plotName); xlabel(ax,'Time (s)'); ylabel(ax,plotUnit); grid(ax,'on');
                else text(0.5,0.5,[plotName,' (Signal Data Not Available)'],'Parent',ax,'HA','center'); end
            end
        end
        if nTotalPlotsForTab4 == 0, text(0.5,0.5,'No valid signals for Env & Flight tab.','Parent',hTabEnvFlight,'HA','center'); end
    catch ME_env_flight
        disp('Error plotting Environment & Flight Conditions Tab:'); disp(getReport(ME_env_flight)); 
    end 

    % --- Tab 5: Forces & Moments --- 
    hTabFM = uitab(hTabGroup, 'Title', 'Forces & Moments');
    try
        fmSignalGroups = {
            {'F_total', {'Fx_{total}', 'Fy_{total}', 'Fz_{total}'}}, ...
            {'M_total', {'Mx_{total}', 'My_{total}', 'Mz_{total}'}}, ...
            {'F_aero', {'Fx_{aero}', 'Fy_{aero}', 'Fz_{aero}'}}, ...
            {'M_aero', {'Mx_{aero}', 'My_{aero}', 'Mz_{aero}'}}, ...
            {'F_thrust', {'Fx_{thrust}', 'Fy_{thrust}', 'Fz_{thrust}'}}, ...
            {'M_thrust', {'Mx_{thrust}', 'My_{thrust}', 'Mz_{thrust}'}}
        };
        axes_fm = { subplot(3,2,1,'Parent',hTabFM), subplot(3,2,3,'Parent',hTabFM), subplot(3,2,5,'Parent',hTabFM), ... 
                    subplot(3,2,2,'Parent',hTabFM), subplot(3,2,4,'Parent',hTabFM), subplot(3,2,6,'Parent',hTabFM) };  
        titles_fm = {'Forces - X Body (N)','Forces - Y Body (N)','Forces - Z Body (N)', 'Moments - X Body (Nm)','Moments - Y Body (Nm)','Moments - Z Body (Nm)'};
        ylabels_fm = {'Force (N)','Force (N)','Force (N)','Moment (Nm)','Moment (Nm)','Moment (Nm)'};
        for i_ax=1:6, title(axes_fm{i_ax},titles_fm{i_ax}); grid(axes_fm{i_ax},'on'); hold(axes_fm{i_ax},'on'); ylabel(axes_fm{i_ax},ylabels_fm{i_ax}); end
        xlabel(axes_fm{3},'Time (s)'); xlabel(axes_fm{6},'Time (s)'); 
        colorOrder = get(groot,'defaultAxesColorOrder');
        
        for k_fm = 1:length(fmSignalGroups)
            sigLogName=fmSignalGroups{k_fm}{1}; compDisplayNames=fmSignalGroups{k_fm}{2}; 
            lineColor = colorOrder(mod(k_fm-1,size(colorOrder,1))+1,:); 
            if any(strcmp(allLoggedSignalNames,sigLogName)) 
                ts_obj = logsout.getElement(sigLogName);
                if isa(ts_obj.Values, 'timeseries')
                    ts=ts_obj.Values; dataToPlot=squeeze(double(ts.Data));
                    if size(dataToPlot,2)==3
                        currentAxesSet = {};
                        if startsWith(sigLogName,'F_'), currentAxesSet=axes_fm(1:3); elseif startsWith(sigLogName,'M_'), currentAxesSet=axes_fm(4:6); end
                        if ~isempty(currentAxesSet)
                            for compIdx=1:3, plot(currentAxesSet{compIdx},ts.Time,dataToPlot(:,compIdx),'DisplayName',compDisplayNames{compIdx},'Color',lineColor); end
                        end
                    elseif isvector(dataToPlot) && ~isempty(dataToPlot) 
                         primaryAxis=[]; 
                         if startsWith(sigLogName,'F_'), primaryAxis=axes_fm{1}; elseif startsWith(sigLogName,'M_'), primaryAxis=axes_fm{4}; end
                         if ~isempty(primaryAxis), plot(primaryAxis,ts.Time,dataToPlot,'DisplayName',strrep(sigLogName,'_',' '),'Color',lineColor); end
                    end
                else
                     fprintf('Signal %s found at top-level but is not a timeseries. Skipping for F&M.\n', sigLogName);
                end
            else
                 fprintf('Signal %s not found in logsout for Forces/Moments tab.\n', sigLogName);
            end
        end
        for i_ax=1:6, legend(axes_fm{i_ax},'show','Location','best','Interpreter','tex'); hold(axes_fm{i_ax},'off'); end
    catch ME_fm
        disp('Error plotting Forces/Moments Tab:'); disp(getReport(ME_fm)); 
    end 

    % --- Tab 6: Inertia Dynamics ---
    hTabInertia = uitab(hTabGroup, 'Title', 'Inertia Dynamics');
    try
        otherDynamicSigs = {'I', 'dI/dt'};
        for k_od = 1:length(otherDynamicSigs)
            sigNameOD = otherDynamicSigs{k_od};
            axOD = subplot(length(otherDynamicSigs),1,k_od, 'Parent', hTabInertia); 
            title(axOD, strrep(sigNameOD,'_',' ')); xlabel(axOD, 'Time (s)'); grid(axOD, 'on'); 
            if any(strcmp(allLoggedSignalNames, sigNameOD)) 
                tsOD_obj = logsout.getElement(sigNameOD);
                if isa(tsOD_obj.Values, 'timeseries')
                    tsOD = tsOD_obj.Values; dataOD_raw = double(tsOD.Data); 
                    hold(axOD, 'on'); plotMadeOD = false; legendEntriesOD = {};
                    if ndims(dataOD_raw)==3 && all(size(dataOD_raw,[1,2])==[3 3]) && size(dataOD_raw,3) == length(tsOD.Time)
                        compData={squeeze(dataOD_raw(1,1,:)), squeeze(dataOD_raw(2,2,:)), squeeze(dataOD_raw(3,3,:))}; compLabels={'_{xx}','_{yy}','_{zz}'};
                        for i_comp=1:3, plot(axOD,tsOD.Time,compData{i_comp},'DisplayName',[strrep(sigNameOD,'_',' '),compLabels{i_comp}]); legendEntriesOD{end+1}=[strrep(sigNameOD,'_',' '),compLabels{i_comp}]; end; plotMadeOD=true;
                    elseif ndims(dataOD_raw)==2 && size(dataOD_raw,2)>0 && size(dataOD_raw,1) == length(tsOD.Time)
                        for comp=1:size(dataOD_raw,2), compName=sprintf('Comp. %d',comp); plot(axOD,tsOD.Time,dataOD_raw(:,comp),'DisplayName',[strrep(sigNameOD,'_',' '),compName]); legendEntriesOD{end+1}=[strrep(sigNameOD,'_',' '),compName]; end; plotMadeOD=true;
                    elseif isvector(squeeze(dataOD_raw)) && ~isempty(squeeze(dataOD_raw)) && length(squeeze(dataOD_raw)) == length(tsOD.Time)
                        plot(axOD,tsOD.Time,squeeze(dataOD_raw),'DisplayName',strrep(sigNameOD,'_',' ')); legendEntriesOD{end+1}=strrep(sigNameOD,'_',' '); plotMadeOD=true;
                    end
                    if plotMadeOD && ~isempty(legendEntriesOD), legend(axOD,legendEntriesOD,'Location','best','Interpreter','tex'); 
                    else text(0.5,0.5,[sigNameOD,' data unhandled/empty or dim mismatch.'],'Parent',axOD,'HA','center'); end
                    hold(axOD,'off');
                else text(0.5,0.5,[sigNameOD,' not timeseries.'],'Parent',axOD,'HA','center'); end
            else text(0.5,0.5,[sigNameOD,' not found.'],'Parent',axOD,'HA','center'); end
        end
    catch ME_Inertia
        disp('Error plotting Inertia Tab:'); disp(getReport(ME_Inertia)); 
    end

    % --- Tab 7: Mass & CG --- 
    hTabMass = uitab(hTabGroup, 'Title', 'Mass & CG');
    try
        axCG = subplot(2,1,1,'Parent',hTabMass); hold(axCG,'on'); grid(axCG,'on');
        plotCountCG = 0;
        if ~isempty(pd) && isfield(pd,'CG') && isa(pd.CG,'timeseries')
            cg_data=squeeze(double(pd.CG.Data)); 
            if size(cg_data,2)>=1, plot(axCG,pd.CG.Time,cg_data(:,1),'DisplayName','CG_x (body)'); plotCountCG=plotCountCG+1; end
            if size(cg_data,2)>=2, plot(axCG,pd.CG.Time,cg_data(:,2),'DisplayName','CG_y (body)'); plotCountCG=plotCountCG+1; end
            if size(cg_data,2)>=3, plot(axCG,pd.CG.Time,cg_data(:,3),'DisplayName','CG_z (body)'); plotCountCG=plotCountCG+1; end
            if plotCountCG > 0, legend(axCG,'show','Location','best'); end
            title(axCG,'CG Position (Body Frame)'); xlabel(axCG,'Time (s)'); ylabel(axCG,'Position (m)');
        else text(0.5,0.5,'CG data missing.','Parent',axCG,'HA','center'); end
        hold(axCG,'off');
        
        axMass = subplot(2,1,2,'Parent',hTabMass);
        if ~isempty(pd) && isfield(pd,'mass') && isa(pd.mass,'timeseries')
            plot(axMass,pd.mass.Time,squeeze(double(pd.mass.Data)));
            title(axMass,'Total Mass'); xlabel(axMass,'Time (s)'); ylabel(axMass,'Mass (kg)'); grid(axMass,'on');
        else text(0.5,0.5,'Mass data missing.','Parent',axMass,'HA','center'); end
    catch ME_mass
        disp('Error plotting Mass & CG Tab:'); disp(getReport(ME_mass)); 
    end 

catch ME_main 
    disp('An error occurred in plot_simulation_results:'); 
    disp(ME_main.message);
    disp(ME_main.getReport('extended', 'hyperlinks','on'));
end 
disp('--- plot_simulation_results finished ---');
end % END OF MAIN FUNCTION plot_simulation_results
