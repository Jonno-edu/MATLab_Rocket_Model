function [TF_alpha, TF_pitch_rate, TF_accel] = compute_TF(...
    m, V, rho, S, CL_alpha, T, L_arm, I, L_ref, Cm_alpha, Cm_gamma_prime)
    % COMPUTE_MISSILE_TF_SI Calculates pitch-plane TFs for ballistic missile
    % Inputs in SI units:
    %   m               - Mass [kg]
    %   V               - Velocity [m/s]
    %   rho             - Air density [kg/m³]
    %   S               - Reference area [m²]
    %   CL_alpha        - Lift coefficient slope [rad⁻¹]
    %   T               - Thrust [N]
    %   L_arm           - TVC moment arm [m]
    %   I               - Moment of inertia [kg·m²]
    %   L_ref           - Reference length [m]
    %   Cm_alpha        - Pitching moment slope [rad⁻¹]
    %   Cm_gamma_prime  - Pitch damping derivative [rad⁻¹]
    
    % Compute simplified constants (Eqs 26-31 in paper)
    d1 = (rho * V * S) / (2 * m) * CL_alpha + T / (m * V);
    d2 = (T * L_arm) / I;
    d3 = (rho * V^2 * S * L_ref) / (2 * I) * Cm_alpha;
    d4 = (rho * V * S * L_ref^2) / (2 * I) * Cm_gamma_prime;
    d5 = T / (m * V);
    
    % Laplace variable
    s = tf('s');
    
    % Common denominator
    denominator = s^2 + (d1 - d4)*s - (d4*d1 + d3);
    
    % Transfer functions
    TF_alpha = (-d5*s + d4*d5 + d2) / denominator;       % α(s)/δ(s)
    TF_pitch_rate = (d2*s + d1*d2 - d3*d5) / denominator; % θ̇(s)/δ(s)
    TF_accel = V * TF_pitch_rate;                        % aₙ(s)/δ(s)
end
