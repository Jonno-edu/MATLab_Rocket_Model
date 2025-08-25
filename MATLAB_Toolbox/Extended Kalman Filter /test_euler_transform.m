% --- Verification Script for Euler Rate Transformation using Intuitive Cases ---

clc;
clear;
close all;

% --- Define the test cases ---
% Each row is a test case: [phi, theta, P, Q, R]
test_cases = [
    % Case 1: Pure Roll, Wings Level
    0, 0, 1, 0, 0;
    
    % Case 2: Pure Pitch, Wings Level
    0, 0, 0, 1, 0;
    
    % Case 3: Pure Yaw, Banked 45 degrees (The definitive test)
    deg2rad(45), 0, 0, 0, 1;
];

fprintf('--- Euler Rate Transformation Verification using Intuitive Test Cases ---\n\n');

% Loop through each test case
for i = 1:size(test_cases, 1)
    % Extract values for the current test case
    phi_val = test_cases(i, 1);
    theta_val = test_cases(i, 2);
    body_rates_val = test_cases(i, 3:5)'; % Transpose to make it a column vector

    fprintf('--- Test Case %d ---\n', i);
    fprintf('Inputs: phi=%.1f deg, theta=%.1f deg, [P; Q; R]=[%.1f; %.1f; %.1f]\n', ...
            rad2deg(phi_val), rad2deg(theta_val), body_rates_val);
            
    % --- Calculate Expected Outcome based on physics ---
    if i == 1 % Pure Roll
        expected_rates = [1; 0; 0];
        fprintf('Expected Outcome: Pure roll rate. phi_dot = P = 1, others = 0.\n');
    elseif i == 2 % Pure Pitch
        expected_rates = [0; 1; 0];
        fprintf('Expected Outcome: Pure pitch rate. theta_dot = Q = 1, others = 0.\n');
    elseif i == 3 % Pure Yaw, Banked
        % A pure yaw while banked should induce a pitch-down and yaw motion
        expected_rates = [0; -sin(phi_val)*body_rates_val(3); cos(phi_val)*body_rates_val(3)];
        fprintf('Expected Outcome: Yaw while banked induces pitch-down. theta_dot = -sin(phi)*R.\n');
    end

    % --- Define the CORRECT transformation matrix (from Eq. 3.7) ---
    T_correct = [1, sin(phi_val)*tan(theta_val), cos(phi_val)*tan(theta_val);
                 0, cos(phi_val),                -sin(phi_val);
                 0, sin(phi_val)*sec(theta_val), cos(phi_val)*sec(theta_val)];

    % --- Define the INCORRECT transformation matrix (from doc Eq. 11.8) ---
    T_incorrect = [1, sin(phi_val)*tan(theta_val), cos(phi_val)*tan(theta_val);
                   0, cos(phi_val),                -sin(theta_val); % TYPO
                   0, sin(phi_val)*sec(theta_val), cos(phi_val)*sec(theta_val)];

    % --- Calculate Euler rates using both matrices ---
    calc_rates_correct = T_correct * body_rates_val;
    calc_rates_incorrect = T_incorrect * body_rates_val;
    
    % --- Display Results ---
    results_table = table(expected_rates, calc_rates_correct, calc_rates_incorrect);
    disp(results_table);
    
    % --- Verdict ---
    if all(abs(expected_rates - calc_rates_correct) < 1e-10)
        fprintf('Verdict: The CORRECT matrix matches the expected physical outcome.\n\n');
    else
        fprintf('Verdict: The CORRECT matrix DOES NOT match the expected outcome.\n\n');
    end
    
    if all(abs(expected_rates - calc_rates_incorrect) < 1e-10)
        fprintf('Verdict: The INCORRECT matrix matches the expected physical outcome.\n\n');
    else
        fprintf('Verdict: The INCORRECT matrix DOES NOT match the expected outcome.\n\n');
    end
    
end
