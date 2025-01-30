function [XDOT] = RCAM_model(X, U)

%STATE AND CONTROL VECTOR
%Extract state vector
x1 = X(1); % u - body-axis velocity component along x-axis
x2 = X(2); % v - body-axis velocity component along y-axis
x3 = X(3); % w - body-axis velocity component along z-axis
x4 = X(4); % p - roll rate
x5 = X(5); % q - pitch rate
x6 = X(6); % r - yaw rate
x7 = X(7); % phi - roll angle
x8 = X(8); % theta - pitch angle
x9 = X(9); % psi - yaw angle

%Extract control vectors
u1 = U(1); % d_A y axis nozzle pitch
u2 = U(2); % d_A z axis nozzle pitch
u3 = U(3); % d_A x axis roll deflection
u4 = U(4); % d_A thrust

%CONSTANTS
m = 5; % Rocket total mass (kg)
S = ; % Aerodynamic reference area (m^2)
d = ; % Aerodynamic reference length of the body (m)



%Centre of gravity coordinates
l_p = ; % The distance from aerodynamic centre to centre of mass
l = ; % Distance from centre of mass to nozzle


%Environmental constants
rho = 1.225; % Air density (kg/m^3)
g = 9.81; % Gravitational accelleration (m/s^2)


%CONTROL LIMITS/SATURATION
ulmin1 = -10 * pi / 180; % Min x axit nozzle pitch (rad)
ulmax1 = 10 * pi / 180; % Max x axit nozzle pitch (rad)
ulmin2 = -10 * pi / 180; % Min y axit nozzle pitch (rad)
ulmax2 = 10 * pi / 180; % Max y axit nozzle pitch (rad)
ulmin3 = 0; % Min z axit roll pitch (rad)
ulmax3 = 0; % Max z axit roll pitch (rad)
ulmin4 = 0; % Min Thrust (N)
ulmax4 = 500; % Min Thrust (N)

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

if (u3 > ulmax3)
    u3 = ulmax3;
elseif (u3 < ulmin3)
    u3 = ulmin3;
end

if (u4 > ulmax4)
    u4 = ulmax4;
elseif (u4 < ulmin4)
    u4 = ulmin4;
end


% INTERMEDIATE VARIABLES


%Define vectors wbe_b and V_b
wbe_b = [x4; x5; x6]; % Angular velocity vector in body frame
V_b = [x1; x2; x3]; % Velocity vector in body frame



% DIMENSIONAL AERODYNAMIC FORCES


% Rotate these forces to F_b (Body axis)
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];

FA_b = C_bs * FA_s; % Aerodynamic force vector FA expressed in body frame (N)


% THRUST FORCE AND MOMENT
%Thrust
gamma1 = u1;
gamma2 = u2;

Fp = u4; % Thrust (N)

Fp_b = Fp * [cos(gamma2) * cos(gamma1); 
             cos(gamma2) * sin(gamma1); 
             -sin(gamma2)];

% GRAVITY EFFECTS
Fg_b = ; % Gravitational force vector Fg expressed in body frame (N)


%Form F_b (all forces in Fb) and calc udot, vdot, wdot
F_b = FA_b + Fg_b + Fp_b; % Aerodynamic force vector, 

x1to3dot = (1 / m) * F_b - cross(wbe_b, V_b); % Translational accelerations

