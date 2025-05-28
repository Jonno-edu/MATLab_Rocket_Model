clear
clc
close all

%% ============================================================================
%% TVC ROCKET - COMPLETE 50% PARAMETER UNCERTAINTY ANALYSIS
%% ============================================================================

%% UNCERTAIN PARAMETERS - Including aerodynamic coefficients
% Physical parameters
V = ureal('V', 392, 'Percentage', 50);
rho = ureal('rho', 0.038, 'Percentage', 50);
m = ureal('m', 974, 'Percentage', 50);
T = ureal('T', 27.6e3, 'Percentage', 50);
I = ureal('I', 19000, 'Percentage', 50);

% AERODYNAMIC COEFFICIENTS - These are estimated and highly uncertain!
CLa = ureal('CLa', 2.0, 'Percentage', 50);      % Lift coefficient uncertainty
Cma = ureal('Cma', 0.885, 'Percentage', 50);    % Moment coefficient uncertainty  
Cmq = ureal('Cmq', -1.05, 'Percentage', 50);    % Pitch damping uncertainty

% Keep geometric parameters fixed
S_ref = 0.200296; L = 9.542; L_arm = 4;

fprintf('=== COMPLETE 50%% PARAMETER UNCERTAINTY ===\n');
fprintf('Physical parameters varied by ±50%%:\n');
fprintf('  V, rho, m, T, I\n');
fprintf('Aerodynamic coefficients varied by ±50%%:\n');
fprintf('  CLa, Cma, Cmq\n\n');

%% BUILD UNCERTAIN PLANT
% Calculate uncertain d-constants (now ALL are uncertain!)
d1 = (rho * V * S_ref)/(2 * m) * CLa + T/(m * V);
d2 = T * L_arm / I;
d3 = (rho * V^2 * S_ref * L)/(2 * I) * Cma;
d4 = (rho * V * S_ref * L^2)/(2 * I) * Cmq;
d5 = T / (m * V);

% Uncertain plant matrices
A = [-d1  0   1;
      0   0   1;
      d3  0   d4];
B = [-d5; 0; d2];

G_uncertain = ss(A, B, eye(3), 0);

% CORRECT: System has 1 input (control), 3 outputs (states)
G_uncertain.InputName = {'δ_control'};  % Single control input
G_uncertain.OutputName = {'α', 'θ', 'θ̇'};  % Three state outputs

fprintf('System structure: 1 input (control), 3 outputs (states)\n');
fprintf('Uncertain d-constants:\n');
fprintf('  d1 (velocity damping + control): involves V, rho, m, CLa, T\n');
fprintf('  d3 (static stability): involves V, rho, I, Cma\n');
fprintf('  d4 (pitch damping): involves V, rho, I, Cmq\n\n');

%% NOMINAL LQR CONTROLLER
A_nom = G_uncertain.NominalValue.A;
B_nom = G_uncertain.NominalValue.B;

Q = diag([10, 100, 1]);
R = 100;
[K, ~] = lqr(A_nom, B_nom, Q, R);
K_controller = ss([], [], [], K);

fprintf('Nominal LQR gains: K = [%.3f, %.3f, %.3f]\n', K);

%% CLOSED-LOOP SYSTEM
T_cl = feedback(G_uncertain * K_controller, eye(3));

%% ALPHA DISTURBANCE STEP RESPONSE
fprintf('\n=== ALPHA DISTURBANCE ANALYSIS ===\n');

% Step response to control input affecting all states
figure;
step(T_cl, 8);
title('Step Control Input Response - 50% Uncertainty (All Parameters)');
grid on;

%% REALISTIC BALLISTIC MISSILE ANALYSIS
% Single control input (jet deflector), multiple state responses

% Your original plant: 1 input (control), 3 outputs (states)
figure;
step(G_uncertain, 8);  % Physical: deflector step → state responses
title('Physical Reality: Jet Deflector Step → All State Responses');

%% OR: Disturbance analysis (what you originally wanted)
% How states respond to external disturbances

% Build disturbance model: disturbances enter states directly
A_cl = G_uncertain.NominalValue.A - G_uncertain.NominalValue.B * K;
Gdist = ss(A_cl, eye(3), eye(3), zeros(3,3));
Gdist.InputName = {'α_disturbance', 'θ_disturbance', 'θ̇_disturbance'};
Gdist.OutputName = {'α', 'θ', 'θ̇'};

% Now just α disturbance response
figure;
step(Gdist(:,1), 8);  %Only α disturbance → all state responses
title('α Disturbance Response (3 plots, not 9)');
