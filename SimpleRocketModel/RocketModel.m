function [XDOT] = RocketModel(X, U)

% Visualization of Both Frames:
% ---------------------------------
%
%   Earth Frame (Inertial)
%         ^ Y (Up, Altitude)
%         |  
%         |  
%         o-------> X (East)
%tes
%   Body Frame (rotates with rocket)
%          ^
%      Y_b |   
%          |-----> X_b  (Rocket forward)
%

x1 = X(1); % u - velocity along body x-axis
x2 = X(2); % v - velocity along body y-axis
x5 = X(3); % q - pitch rate
x8 = X(4); % theta - pitch angle


%Extract control vectors
u1 = U(1); % d_A thrust
u2 = U(2); % d_A z axis nozzle pitch

%CONSTANTS
m = 5; % Rocket total mass (kg)


%Centre of gravity coordinates
l_p = 1; % The distance from aerodynamic centre to centre of mass
l = 1.5; % Distance from centre of mass to nozzle


%Environmental constants
rho = 1.225; % Air density (kg/m^3)
g = 9.81; % Gravitational accelleration (m/s^2)


%CONTROL LIMITS/SATURATION
ulmin1 = 0; % min thrust (N)
ulmax1 = 200; % max thrust (N)
ulmin2 = -10 * pi / 180; % Min z axit nozzle pitch (rad)
ulmax2 = 10 * pi / 180; % Max z axit nozzle pitch (rad)


% Saturate control inputs
if (u1 > ulmax1)
    u1 = ulmax1;
elseif (u1 < ulmin1)
    u1 = ulmin1;
end

if (u2 > ulmax2)
    u2 = ulmax2;
elseif (u2 < ulmin2)
    u2 = ulmin2;
end


% INTERMEDIATE VARIABLES


%Define vectors wbe_b and V_b
wbe_b = [x5]; % Angular velocity vector in body frame
V_b = [x1; x2]; % Velocity vector in body frame


%Inertia Matrix for 2D
Ib = m * 40.07; % Simplified inertia for 2D (example value)
invIb = 1 / Ib; % Inverse of inertia


% THRUST FORCE AND MOMENT
%Thrust
gamma1 = u1;

Fp = u1; % Thrust (N)

Fp_b = Fp * [cos(gamma1); 
             sin(gamma1)];

% GRAVITY EFFECTS
g_b = [-g * sin(x8);
        g * cos(x8)];

Fg_b = m * g_b; % Gravitational force in body frame


%Form F_b (all forces in Fb) and calc udot, vdot, wdot
F_b = Fg_b + Fp_b; % Gravity force vector, Thrust force vector

% Simplified cross product for 2D
cross_wbe_b_V_b = wbe_b * V_b(2) - wbe_b * V_b(1);

% Split x1to3dot into separate calculations
udot = (1 / m) * F_b(1) - cross_wbe_b_V_b; % Translational acceleration in x direction
vdot = (1 / m) * F_b(2) - cross_wbe_b_V_b; % Translational acceleration in y direction

x1to3dot = [udot; vdot]; % Combine into a vector

%Form Mcg_b (All moments about CoG in Fb) and calc qdot
Mcg_b = l_p * Fp_b(2); % Moment due to thrust (simplified for 2D)
x4dot = invIb * (Mcg_b - wbe_b * Ib * wbe_b); % Rotational acceleration (qdot)

%Calculate thetadot
x5dot = x5; % Angular rate (thetadot)

%Place in first order form
XDOT = [x1to3dot;
        x4dot;
        x5dot]; % State derivatives

