%% Initialization (as you already have)
T    = 30722;          % Thrust [lbf]
V    = 1285;           % Velocity [ft/s]
q    = 585;            % Dynamic pressure [lb/ft^2]
rho  = 2*q/(V^2);      % Air density [slug/ft^3]
S_area    = 11;        % Reference area [ft^2]
L    = 3.75;           % Reference length [ft]
m    = 445;            % Mass [slugs]
I    = 115000;         % Moment of inertia [slug*ft^2]

% Aerodynamic derivatives
C_al        = -3.13;   % dCL/dα [1/rad]
C_Ldelta    = -4.63;   % dCL/dδ [1/rad]
Cm_alpha    = 11.27;   % dCm/dα [1/rad]
Cm_delta    = -34.25;  % dCm/dδ [1/rad]
Cm_gammadot = -220;    % dCm/d(γ′) [–]
Cm_alphadot =   0;     % dCm/d(α′) [–]

%% Compute lumped constants
d1 = rho*V*S_area/(2*m)*C_al   + T/(m*V);
d2 = rho*V^2*S_area*L/(2*I) * Cm_delta;
d3 = rho*V^2*S_area*L/(2*I) * Cm_alpha;
d4 = rho*V*S_area*L^2/(2*I) * Cm_gammadot;
d5 = rho*V*S_area/(2*m)     * C_Ldelta;
d6 = rho*V*S_area*L^2/(2*I) * Cm_alphadot;

%% Transfer function (corrected)
s = tf('s');

alpha_nozzleAngle_TF = ...
    (-d5*s + d4*d5 + d2) ...
    / (s^2 + (d1 - d4 - d6)*s - (d4*d1 + d3))

gamma_nozzleAngle_TF = ((d2-d5*d6)*s + d2*d1 - d5*d3)...
                        / (s^2 + (d1 - d4 - d6)*s - (d4*d1 + d3))



%% Root locus
%rlocus(alpha_nozzleAngle_TF)
rlocus(gamma_nozzleAngle_TF)
