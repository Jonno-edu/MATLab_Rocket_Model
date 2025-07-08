
% Paper parameters converted to SI units
m = 6494;               % Mass [kg] (445 slugs → 6494 kg)
V = 391.7;              % Velocity [m/s] (1285 ft/s → 391.7 m/s)
rho = 0.413;            % Air density [kg/m³] at 36k ft (~11 km)
S = 1.022;              % Reference area [m²] (11 ft² → 1.022 m²)
CL_alpha = 3.13;        % Lift coefficient slope [rad⁻¹]
T = 136670;             % Thrust [N] (30,722 lbf → 136,670 N)
L_arm = 1.143;          % Moment arm [m] (3.75 ft → 1.143 m)
I = 155930;             % Moment of inertia [kg·m²] (115k slug·ft² → 155,930 kg·m²)
L_ref = 1.143;          % Reference length [m] (3.75 ft → 1.143 m)
Cm_alpha = 11.27;       % Pitching moment slope [rad⁻¹]
Cm_gamma_prime = -4.63; % Pitch damping derivative [rad⁻¹]

% Compute transfer functions
[TF_alpha_TVC, TF_pitch_rate_TVC, TF_accel_TVC] = compute_TF(...
    m, V, rho, S, CL_alpha, T, L_arm, I, L_ref, Cm_alpha, Cm_gamma_prime);

% Display results
disp('Angle-of-Attack TF (α/δ):');
TF_alpha_TVC
disp('Pitch-Rate TF (θ̇/δ):');
TF_pitch_rate_TVC
disp('Normal Acceleration TF (aₙ/δ):');
TF_accel_TVC






