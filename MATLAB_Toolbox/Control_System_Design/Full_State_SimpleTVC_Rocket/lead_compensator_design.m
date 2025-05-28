clear
clc
close all

%% Plant model (TVC Rocket - Just After Launch Conditions)
% Physical parameters (SI units) - Just after launch scenario
m = 974;             % rocket mass (kg) - still has most fuel
rho = 0.038;         % air density (kg/m³) - sea level
V = 392;             % velocity (m/s) - 3-5 seconds after launch, accelerating
S = 0.200296;        % reference area (m²)
L = 9.542;           % reference length (m)
I = 19000;           % moment of inertia about pitch axis (kg⋅m²)
T = 27.6*10^3;       % thrust (N)
L_arm = 4;           % TVC moment arm from CG to nozzle (m)

% Aerodynamic coefficients (from your calculations)
CLa = 2.0;           % lift curve slope (1/rad) - from your data
Cma = 0.885;         % moment curve slope (1/rad) - from CP-CG calculation
Cmq = -1.05;         % pitch damping coefficient (1/rad) - from your function

% Dynamic pressure
q = 0.5 * rho * V^2;  % dynamic pressure (Pa or N/m²)
fprintf('=== JUST AFTER LAUNCH CONDITIONS ===\n');
fprintf('Velocity: %d m/s\n', V);
fprintf('Altitude: Sea level (rho = %.1f kg/m³)\n', rho);
fprintf('Dynamic pressure: %.0f Pa\n', q);

% Calculate d constants (SI units)
d1 = (rho * V * S)/(2 * m) * CLa + T/(m * V);      % [1/s]
d2 = T * L_arm / I;                                 % [rad/s² per rad]
d3 = (rho * V^2 * S * L)/(2 * I) * Cma;           % [rad/s² per rad]  
d4 = (rho * V * S * L^2)/(2 * I) * Cmq;           % [1/s]
d5 = T / (m * V);                                   % [1/s]

% Display results with units
fprintf('\nPlant d-constants:\n');
fprintf('d1 = %.4f (1/s) - AoA aerodynamic + thrust normal effect\n', d1);
fprintf('d2 = %.4f (rad/s² per rad) - TVC moment authority\n', d2);  
fprintf('d3 = %.4f (rad/s² per rad) - Static stability (positive = unstable)\n', d3);
fprintf('d4 = %.4f (1/s) - Pitch damping (negative = damping)\n', d4);
fprintf('d5 = %.4f (1/s) - Thrust normal force coefficient\n', d5);

%% Basic 3-State Plant (NO ACTUATOR)
% States: [alpha, theta, theta_dot]
% Input: nozzle_deflection_command (direct, no actuator lag)

A_plant = [-d1  0   1;      % alpha_dot equation
            0   0   1;      % theta_dot equation  
            d3  0   d4];    % theta_double_dot equation

B_plant = [-d5;             % TVC effect on alpha_dot
            0;              % No direct effect on theta_dot
            d2];            % TVC effect on theta_double_dot

C_plant = eye(3);           % Output all states
D_plant = zeros(3,1);       % No feedthrough

% Check system properties
open_loop_poles_plant = eig(A_plant);
fprintf('\nBasic plant open-loop poles:\n');
disp(open_loop_poles_plant)

% Check stability
unstable_poles = sum(real(open_loop_poles_plant) > 0);
fprintf('Number of unstable poles: %d\n', unstable_poles);

%% Verify Controllability
Pc_plant = ctrb(A_plant, B_plant);
rank_plant = rank(Pc_plant);
fprintf('Plant controllability rank: %d (should be 3)\n', rank_plant);

if rank_plant < 3
    fprintf('WARNING: Plant is not controllable!\n');
    return;
end

%% STABILITY ANALYSIS FUNCTIONS
% Comprehensive stability analysis including MIMO and individual state analysis

function results = comprehensive_stability_analysis(A, B, C, K, design_name)
    % Comprehensive stability analysis with robust error handling
    
    fprintf('\n=== %s STABILITY ANALYSIS ===\n', design_name);
    
    % Initialize results structure
    results = struct();
    results.design_name = design_name;
    results.K = K;
    
    % 1. BASIC CLOSED-LOOP STABILITY
    A_cl = A - B*K;
    poles_cl = eig(A_cl);
    results.poles = poles_cl;
    results.basic_stable = all(real(poles_cl) < -1e-6);
    
    fprintf('1. Basic Pole Analysis:\n');
    fprintf('   Poles: ');
    for i = 1:length(poles_cl)
        if abs(imag(poles_cl(i))) < 1e-6
            fprintf('%.4f ', real(poles_cl(i)));
        else
            fprintf('%.4f±%.4fj ', real(poles_cl(i)), abs(imag(poles_cl(i))));
        end
    end
    fprintf('\n');
    fprintf('   Basic stability: %s\n', string(results.basic_stable));
    
    % 2. CLASSICAL SISO-STYLE MARGINS (potentially misleading for MIMO)
    fprintf('\n2. Classical SISO-Style Margins (potentially misleading):\n');
    try
        % Create loop transfer function for margin analysis
        sys_plant = ss(A, B, C, zeros(size(C,1), size(B,2)));
        
        % Method 1: Try direct K*G approach
        if size(K,1) == 1 && size(sys_plant.C,1) > 1
            % Single input, multiple outputs - use first output for classical analysis
            G_siso = ss(A, B, C(1,:), 0);  % Use alpha (first state) for classical analysis
            L_classical = K * G_siso;
        else
            L_classical = K * sys_plant;
        end
        
        [Gm_classical, Pm_classical] = margin(L_classical);
        
        if ~isinf(Gm_classical)
            fprintf('   Overall: GM=%.2f dB, PM=%.1f°\n', ...
                    20*log10(Gm_classical), Pm_classical);
            results.classical_gm_db = 20*log10(Gm_classical);
        else
            fprintf('   Overall: GM=Infinite, PM=%.1f°\n', Pm_classical);
            results.classical_gm_db = 999;
        end
        results.classical_pm = Pm_classical;
        
    catch ME
        fprintf('   Classical margin calculation failed: %s\n', ME.message);
        results.classical_gm_db = NaN;
        results.classical_pm = NaN;
    end
    
    % 3. INDIVIDUAL STATE ANALYSIS (Loop-at-a-Time)
    fprintf('\n3. Individual State Analysis (Loop-at-a-Time):\n');
    state_names = {'Alpha (AoA)', 'Theta (pitch)', 'Theta_dot (pitch rate)'};
    results.individual_margins = struct();
    
    for i = 1:3
        fprintf('   %s:\n', state_names{i});
        
        try
            % Extract transfer function from input to state i
            C_i = zeros(1,3);
            C_i(i) = 1;
            G_i = ss(A, B, C_i, 0);
            
            % Individual loop transfer function
            L_i = K(i) * G_i;
            
            % Calculate margins for this state
            [Gm_i, Pm_i] = margin(L_i);
            
            if ~isinf(Gm_i)
                gm_i_db = 20*log10(Gm_i);
                fprintf('     GM=%.2f dB, PM=%.1f°\n', gm_i_db, Pm_i);
            else
                gm_i_db = 999;
                fprintf('     GM=Infinite, PM=%.1f°\n', Pm_i);
            end
            
            % Store individual results
            results.individual_margins.(sprintf('state_%d', i)) = struct(...
                'name', state_names{i}, 'gm_db', gm_i_db, 'pm', Pm_i);
            
        catch ME
            fprintf('     Analysis failed: %s\n', ME.message);
            results.individual_margins.(sprintf('state_%d', i)) = struct(...
                'name', state_names{i}, 'gm_db', NaN, 'pm', NaN);
        end
    end
    
    % 4. MIMO DISK MARGINS (Most Reliable) - with robust error handling
    fprintf('\n4. MIMO Disk Margins (Most Reliable):\n');
    results.mimo_analysis_available = false;
    
    try
        % Create proper MIMO loop transfer function
        sys_plant_mimo = ss(A, B, C, zeros(size(C,1), size(B,2)));
        L_mimo = K * sys_plant_mimo;
        
        % Try multiple MIMO analysis approaches
        mimo_success = false;
        
        % Approach 1: Try allmargin function
        try
            if exist('allmargin', 'file')
                [SM, MM] = allmargin(L_mimo);
                
                results.mimo_margins = MM;
                results.disk_margin = MM.DiskMargin;
                results.mimo_analysis_available = true;
                
                fprintf('   MIMO Gain Margins: [%.4f, %.4f] (linear scale)\n', MM.GainMargin);
                fprintf('   MIMO Phase Margins: [%.2f, %.2f] degrees\n', MM.PhaseMargin);
                fprintf('   Disk Margin: %.4f\n', MM.DiskMargin);
                fprintf('   Critical Frequency: %.4f rad/s\n', MM.Frequency);
                
                % Interpret disk margin
                if MM.DiskMargin > 1.0
                    robustness_assessment = 'Excellent';
                    fprintf('   ✓ Excellent MIMO robustness\n');
                elseif MM.DiskMargin > 0.5
                    robustness_assessment = 'Good';
                    fprintf('   ✓ Good MIMO robustness\n');
                elseif MM.DiskMargin > 0.2
                    robustness_assessment = 'Marginal';
                    fprintf('   ⚠ Marginal MIMO robustness\n');
                else
                    robustness_assessment = 'Poor';
                    fprintf('   ✗ Poor MIMO robustness\n');
                end
                
                results.robustness_assessment = robustness_assessment;
                mimo_success = true;
            end
        catch ME1
            fprintf('   allmargin failed: %s\n', ME1.message);
        end
        
        % Approach 2: Try diskmargin function (if allmargin failed)
        if ~mimo_success
            try
                if exist('diskmargin', 'file')
                    [DM, MM_disk] = diskmargin(L_mimo);
                    
                    results.disk_margin = DM.DiskMargin;
                    results.mimo_analysis_available = true;
                    
                    fprintf('   Disk Margin: %.4f\n', DM.DiskMargin);
                    fprintf('   Gain Margin: [%.4f, %.4f] (linear)\n', DM.GainMargin);
                    fprintf('   Phase Margin: [%.2f, %.2f] degrees\n', DM.PhaseMargin);
                    
                    if DM.DiskMargin > 0.5
                        robustness_assessment = 'Good';
                        fprintf('   ✓ Good MIMO robustness\n');
                    elseif DM.DiskMargin > 0.2
                        robustness_assessment = 'Marginal';
                        fprintf('   ⚠ Marginal MIMO robustness\n');
                    else
                        robustness_assessment = 'Poor';
                        fprintf('   ✗ Poor MIMO robustness\n');
                    end
                    
                    results.robustness_assessment = robustness_assessment;
                    mimo_success = true;
                end
            catch ME2
                fprintf('   diskmargin failed: %s\n', ME2.message);
            end
        end
        
        % Approach 3: Manual MIMO analysis (if both above failed)
        if ~mimo_success
            fprintf('   Advanced MIMO functions not available\n');
            fprintf('   Performing basic MIMO analysis...\n');
            
            try
                % Basic MIMO stability check using determinant condition
                freq_range = logspace(-1, 2, 50);
                det_margins = zeros(size(freq_range));
                
                for f_idx = 1:length(freq_range)
                    s_val = 1j * freq_range(f_idx);
                    L_val = freqresp(L_mimo, freq_range(f_idx));
                    
                    % For SISO case
                    if size(L_val, 1) == 1 && size(L_val, 2) == 1
                        det_margins(f_idx) = abs(1 + L_val);
                    else
                        % For MIMO case
                        det_margins(f_idx) = abs(det(eye(size(L_val)) + L_val));
                    end
                end
                
                min_det = min(det_margins);
                results.min_det_margin = min_det;
                
                fprintf('   Minimum determinant margin: %.4f\n', min_det);
                if min_det > 0.5
                    fprintf('   ✓ Adequate MIMO stability\n');
                    results.robustness_assessment = 'Adequate';
                elseif min_det > 0.1
                    fprintf('   ⚠ Marginal MIMO stability\n');
                    results.robustness_assessment = 'Marginal';
                else
                    fprintf('   ✗ Poor MIMO stability\n');
                    results.robustness_assessment = 'Poor';
                end
                
            catch ME3
                fprintf('   Basic MIMO analysis failed: %s\n', ME3.message);
                results.robustness_assessment = 'Unable to analyze';
            end
        end
        
    catch ME
        fprintf('   All MIMO analysis methods failed: %s\n', ME.message);
        results.disk_margin = NaN;
        results.robustness_assessment = 'Analysis failed';
    end
    
    % 5. ADDITIONAL ROBUSTNESS TOOLS
    fprintf('\n5. Additional Robustness Analysis:\n');
    
    % Singular value analysis
    try
        sys_plant_sv = ss(A, B, C, zeros(size(C,1), size(B,2)));
        L_sv = K * sys_plant_sv;
        freq_sv = logspace(-1, 2, 100);
        sv_vals = sigma(L_sv, freq_sv);
        results.max_singular_value = max(sv_vals(1,:));
        results.min_singular_value = min(sv_vals(end,:));
        fprintf('   Singular value analysis completed\n');
    catch
        fprintf('   Singular value analysis failed\n');
    end
    
    % Condition number analysis
    try
        cond_A = cond(A);
        cond_controllability = cond(ctrb(A, B));
        
        results.condition_A = cond_A;
        results.condition_controllability = cond_controllability;
        
        fprintf('   System matrix condition number: %.2e\n', cond_A);
        fprintf('   Controllability matrix condition: %.2e\n', cond_controllability);
        
        if cond_controllability > 1e12
            fprintf('   ⚠ Poor controllability conditioning\n');
        else
            fprintf('   ✓ Acceptable controllability conditioning\n');
        end
        
    catch
        fprintf('   Condition number analysis failed\n');
    end
    
    % 6. SUMMARY ASSESSMENT
    fprintf('\n6. Overall Assessment:\n');
    
    % Determine overall system status
    if results.basic_stable
        fprintf('   ✓ Basic stability: PASSED\n');
        
        if results.mimo_analysis_available && isfield(results, 'disk_margin') && ~isnan(results.disk_margin)
            if results.disk_margin > 0.2
                fprintf('   ✓ MIMO robustness: ADEQUATE\n');
                overall_status = 'ACCEPTABLE';
            else
                fprintf('   ⚠ MIMO robustness: POOR\n');
                overall_status = 'MARGINAL';
            end
        else
            overall_status = 'STABILITY_ONLY';
        end
    else
        fprintf('   ✗ Basic stability: FAILED\n');
        overall_status = 'UNSTABLE';
    end
    
    results.overall_status = overall_status;
    fprintf('   Overall Status: %s\n', overall_status);
end

%% COMPREHENSIVE LQR OPTIMIZATION WITH STABILITY ANALYSIS

fprintf('\n=== COMPREHENSIVE LQR OPTIMIZATION WITH STABILITY ANALYSIS ===\n');

% Define search ranges for Q diagonal elements (smaller range for detailed analysis)
alpha_max_range = [0.5, 1, 2, 5]*pi/180;      % Max acceptable AoA (rad)
theta_max_range = [2, 5, 10, 20]*pi/180;      % Max acceptable pitch (rad)  
theta_dot_max_range = [10, 30, 50, 100]*pi/180; % Max acceptable pitch rate (rad/s)
delta_max_range = [2, 4, 8, 15]*pi/180;       % Max acceptable control (rad)

% Storage for optimization results with comprehensive analysis
opt_results = [];
analysis_results = [];

total_combinations = length(alpha_max_range)*length(theta_max_range)*length(theta_dot_max_range)*length(delta_max_range);
fprintf('Testing %d combinations with comprehensive stability analysis...\n', total_combinations);

% Grid search with comprehensive analysis
search_count = 0;
for i = 1:length(alpha_max_range)
    for j = 1:length(theta_max_range)
        for k = 1:length(theta_dot_max_range)
            for l = 1:length(delta_max_range)
                search_count = search_count + 1;
                
                fprintf('\n--- Design %d/%d ---\n', search_count, total_combinations);
                
                % Construct Q and R matrices using Bryson's rule
                Q_test = diag([1/alpha_max_range(i)^2, 1/theta_max_range(j)^2, 1/theta_dot_max_range(k)^2]);
                R_test = 1/delta_max_range(l)^2;
                
                fprintf('Testing: α_max=%.1f°, θ_max=%.1f°, θ̇_max=%.1f°, δ_max=%.1f°\n', ...
                        alpha_max_range(i)*180/pi, theta_max_range(j)*180/pi, ...
                        theta_dot_max_range(k)*180/pi, delta_max_range(l)*180/pi);
                
                try
                    % Calculate LQR gain
                    K_test = lqr(A_plant, B_plant, Q_test, R_test);
                    
                    % Comprehensive stability analysis
                    design_name = sprintf('Design_%d', search_count);
                    analysis_result = comprehensive_stability_analysis(A_plant, B_plant, C_plant, K_test, design_name);
                    
                    % Store results
                    result = struct('search_id', search_count, ...
                                   'alpha_max', alpha_max_range(i)*180/pi, ...
                                   'theta_max', theta_max_range(j)*180/pi, ...
                                   'theta_dot_max', theta_dot_max_range(k)*180/pi, ...
                                   'delta_max', delta_max_range(l)*180/pi, ...
                                   'Q', Q_test, 'R', R_test, 'K', K_test, ...
                                   'basic_stable', analysis_result.basic_stable, ...
                                   'overall_status', analysis_result.overall_status);
                    
                    % Add stability metrics if available
                    if isfield(analysis_result, 'classical_gm_db')
                        result.classical_gm_db = analysis_result.classical_gm_db;
                    end
                    if isfield(analysis_result, 'disk_margin')
                        result.disk_margin = analysis_result.disk_margin;
                    end
                    
                    opt_results = [opt_results, result];
                    analysis_results = [analysis_results, analysis_result];
                    
                catch ME
                    fprintf('Design failed: %s\n', ME.message);
                    continue;
                end
            end
        end
    end
end

%% COMPARISON AND ANALYSIS OF RESULTS

fprintf('\n\n=== COMPREHENSIVE RESULTS ANALYSIS ===\n');

if ~isempty(opt_results)
    % Extract metrics for analysis
    basic_stable_flags = [opt_results.basic_stable];
    
    % Classical margins (where available)
    classical_gms = [];
    for i = 1:length(opt_results)
        if isfield(opt_results(i), 'classical_gm_db') && ~isnan(opt_results(i).classical_gm_db)
            classical_gms(end+1) = opt_results(i).classical_gm_db;
        else
            classical_gms(end+1) = NaN;
        end
    end
    
    % MIMO disk margins (where available)
    disk_margins = [];
    for i = 1:length(opt_results)
        if isfield(opt_results(i), 'disk_margin') && ~isnan(opt_results(i).disk_margin)
            disk_margins(end+1) = opt_results(i).disk_margin;
        else
            disk_margins(end+1) = NaN;
        end
    end
    
    % Summary statistics
    fprintf('SUMMARY STATISTICS:\n');
    fprintf('Total designs tested: %d\n', length(opt_results));
    fprintf('Basically stable designs: %d (%.1f%%)\n', sum(basic_stable_flags), ...
            sum(basic_stable_flags)/length(opt_results)*100);
    
    % Classical margin statistics
    positive_classical = sum(classical_gms > 0 & ~isnan(classical_gms));
    fprintf('Positive classical gain margins: %d (%.1f%%)\n', positive_classical, ...
            positive_classical/length(opt_results)*100);
    
    % MIMO margin statistics  
    good_mimo = sum(disk_margins > 0.2 & ~isnan(disk_margins));
    fprintf('Good MIMO robustness (disk margin > 0.2): %d (%.1f%%)\n', good_mimo, ...
            good_mimo/length(opt_results)*100);
    
    % Comparison: Individual vs MIMO Analysis
    fprintf('\n=== INDIVIDUAL vs MIMO ANALYSIS COMPARISON ===\n');
    fprintf('This demonstrates the spinning satellite phenomenon from search results:\n');
    
    for i = 1:min(5, length(analysis_results))  % Show first 5 designs
        fprintf('\nDesign %d:\n', i);
        
        % Individual state margins
        if isfield(analysis_results(i), 'individual_margins')
            fprintf('  Individual state margins:\n');
            for state_idx = 1:3
                field_name = sprintf('state_%d', state_idx);
                if isfield(analysis_results(i).individual_margins, field_name)
                    state_data = analysis_results(i).individual_margins.(field_name);
                    if ~isnan(state_data.gm_db)
                        if state_data.gm_db > 100
                            fprintf('    %s: GM=Infinite, PM=%.1f°\n', state_data.name, state_data.pm);
                        else
                            fprintf('    %s: GM=%.1f dB, PM=%.1f°\n', state_data.name, state_data.gm_db, state_data.pm);
                        end
                    end
                end
            end
        end
        
        % MIMO margins
        if isfield(analysis_results(i), 'disk_margin') && ~isnan(analysis_results(i).disk_margin)
            fprintf('  MIMO disk margin: %.4f (%s)\n', analysis_results(i).disk_margin, ...
                    analysis_results(i).robustness_assessment);
        end
        
        % Overall assessment
        fprintf('  Overall status: %s\n', analysis_results(i).overall_status);
    end
    
    %% VISUALIZATION
    fprintf('\n=== CREATING VISUALIZATION ===\n');
    
    figure('Position', [100 100 1400 1000]);
    
    % Plot 1: Classical vs MIMO margins
    subplot(2,3,1);
    valid_idx = ~isnan(classical_gms) & ~isnan(disk_margins);
    if sum(valid_idx) > 0
        scatter(classical_gms(valid_idx), disk_margins(valid_idx), 50, 'filled');
        xlabel('Classical Gain Margin (dB)');
        ylabel('MIMO Disk Margin');
        title('Classical vs MIMO Margins');
        grid on;
        
        % Add reference lines
        xline(0, '--r', 'Classical GM = 0 dB');
        yline(0.2, '--g', 'Good MIMO Robustness');
    else
        text(0.5, 0.5, 'No valid data', 'HorizontalAlignment', 'center');
        title('Classical vs MIMO Margins (No Data)');
    end
    
    % Plot 2: Stability status distribution
    subplot(2,3,2);
    status_types = {opt_results.overall_status};
    [unique_status, ~, status_idx] = unique(status_types);
    status_counts = accumarray(status_idx, 1);
    
    pie(status_counts, unique_status);
    title('Overall Stability Status Distribution');
    
    % Plot 3: Best designs comparison
    subplot(2,3,3);
    if ~isempty(disk_margins) && sum(~isnan(disk_margins)) > 0
        [sorted_disk, sort_idx] = sort(disk_margins, 'descend', 'MissingPlacement', 'last');
        
        % Show top 10 or all available
        n_show = min(10, sum(~isnan(disk_margins)));
        if n_show > 0
            bar(1:n_show, sorted_disk(1:n_show));
            xlabel('Design Rank');
            ylabel('MIMO Disk Margin');
            title('Top Designs by MIMO Robustness');
            grid on;
            
            % Add robustness threshold lines
            yline(0.2, '--g', 'Good Threshold');
            yline(0.5, '--b', 'Excellent Threshold');
        end
    else
        text(0.5, 0.5, 'No MIMO data available', 'HorizontalAlignment', 'center');
        title('MIMO Robustness Ranking (No Data)');
    end
    
    % Plot 4: Parameter sensitivity
    subplot(2,3,4);
    if ~isempty(classical_gms)
        alpha_vals = [opt_results.alpha_max];
        scatter(alpha_vals, classical_gms, 50, 'filled');
        xlabel('Alpha Max (degrees)');
        ylabel('Classical Gain Margin (dB)');
        title('Sensitivity to Alpha Tolerance');
        grid on;
    end
    
    % Plot 5: Control effort analysis
    subplot(2,3,5);
    delta_vals = [opt_results.delta_max];
    scatter(delta_vals, classical_gms, 50, 'filled');
    xlabel('Control Limit (degrees)');
    ylabel('Classical Gain Margin (dB)');
    title('Control Authority vs Stability');
    grid on;
    
    subplot(2,3,6);
    text(0.1, 0.8, sprintf('LQR Classical GM Range: %.1f to %.1f dB', min(classical_gms), max(classical_gms)), 'FontSize', 12);
    
    if ~isempty(disk_margins) && sum(~isnan(disk_margins)) > 0
        improvement_text = sprintf('%.4f', max(disk_margins) - min(disk_margins(~isnan(disk_margins))));
    else
        improvement_text = 'N/A';
    end
    
    text(0.1, 0.6, sprintf('MIMO Disk Margin Range: %s', improvement_text), 'FontSize', 12);
    text(0.1, 0.4, sprintf('Best Overall Status: %s', opt_results(1).overall_status), 'FontSize', 12);
    title('Summary Statistics');
    axis off;
    
    sgtitle('Comprehensive Stability Analysis Results', 'FontSize', 14);
    
    %% RECOMMENDATIONS
    fprintf('\n=== RECOMMENDATIONS BASED ON ANALYSIS ===\n');
    
    % Find best design by different criteria
    if sum(~isnan(disk_margins)) > 0
        [best_mimo_margin, best_mimo_idx] = max(disk_margins);
        fprintf('Best MIMO robustness design:\n');
        fprintf('  Design ID: %d\n', best_mimo_idx);
        fprintf('  MIMO disk margin: %.4f\n', best_mimo_margin);
        fprintf('  Overall status: %s\n', opt_results(best_mimo_idx).overall_status);
    end
    
    if sum(basic_stable_flags) > 0
        fprintf('\nNumber of stable designs found: %d\n', sum(basic_stable_flags));
        
        if sum(~isnan(disk_margins)) == 0
            fprintf('WARNING: No MIMO analysis succeeded - may need Control System Toolbox\n');
        elseif max(disk_margins(~isnan(disk_margins))) < 0.2
            fprintf('WARNING: All designs have poor MIMO robustness\n');
        end
    else
        fprintf('\nWARNING: No stable designs found in search space\n');
    end
    
    fprintf('\n=== EDUCATIONAL SUMMARY ===\n');
    fprintf('This analysis demonstrates key concepts:\n');
    fprintf('1. Individual state margins can be misleading for MIMO systems\n');
    fprintf('2. MIMO disk margins provide more reliable robustness assessment\n');
    fprintf('3. Basic pole stability ≠ robust performance\n');
    fprintf('4. Classical gain margins consistently negative = fundamental challenge\n');
    
else
    fprintf('No valid designs found in optimization!\n');
end

fprintf('\n=== ANALYSIS COMPLETE ===\n');
