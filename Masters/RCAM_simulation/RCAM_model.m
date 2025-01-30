function [XDOT] = RCAM_model(X, U)
% Function to compute the state derivatives for the RCAM model

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

%Extract control vector
u1 = U(1); % d_A - aileron deflection
u2 = U(2); % d_T - stabilizer deflection
u3 = U(3); % d_R - rudder deflection
u4 = U(4); % d_th1 - throttle 1
u5 = U(5); % d_th2 - throttle 2

%CONSTANTS
m = 120000; % Aircraft total mass (kg)
cbar = 6.6; % Mean aerodynamic chord (m)
lt = 24.8; % Tail length (m)
S = 260; % Wing area (m^2)
St = 64; % Tail area (m^2)

% Center of gravity coordinates
Xcg = 0.23 * cbar;
Ycg = 0;
Zcg = 0.1 * cbar;

% Aerodynamic center coordinates
Xac = 0.12 * cbar;
Yac = 0;
Zac = 0;

% Engine coordinates
Xapt1 = 0;
Yapt1 = -7.94;
Zapt1 = -1.9;

Xapt2 = 0;
Yapt2 = 7.94;
Zapt2 = -1.9;

% Environmental constants
rho = 1.225; % Air density (kg/m^3)
g = 9.81; % Gravitational acceleration (m/s^2)
depsa = 0.25; % Downwash gradient
alpha_LO = -11.5 * pi / 180; % Lift-off angle of attack (rad)
n = 5.5; % Lift curve slope
a3 = -768.5; % Coefficient for cubic term in CL-alpha curve
a2 = 609.2; % Coefficient for quadratic term in CL-alpha curve
a1 = -155.2; % Coefficient for linear term in CL-alpha curve
a0 = 15.212; % Constant term in CL-alpha curve
alpha_switch = 14.5 * (pi / 180); % Angle of attack switch (rad)

%CONTROL LIMITS/SATURATION
ulmin1 = -25 * pi / 180; % Minimum aileron deflection (rad)
ulmax1 = 25 * pi / 180; % Maximum aileron deflection (rad)
ulmin2 = -25 * pi / 180; % Minimum stabilizer deflection (rad)
ulmax2 = 10 * pi / 180; % Maximum stabilizer deflection (rad)
ulmin3 = -30 * pi / 180; % Minimum rudder deflection (rad)
ulmax3 = 30 * pi / 180; % Maximum rudder deflection (rad)
ulmin4 = 0.5 * pi / 180; % Minimum throttle 1 (rad)
ulmax4 = 10 * pi / 180; % Maximum throttle 1 (rad)
ulmin5 = 0.5 * pi / 180; % Minimum throttle 2 (rad)
ulmax5 = 10 * pi / 180; % Maximum throttle 2 (rad)

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

if (u5 > ulmax5)
    u5 = ulmax5;
elseif (u5 < ulmin5)
    u5 = ulmin5;
end

%2. INTERMEDIATE VARIABLES
%Calculate airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

%Calculate alpha and beta
alpha = atan2(x3, x1); % Angle of attack
beta = asin(x2 / Va); % Sideslip angle

%Calculate Dynamic pressure
Q = 0.5 * rho * Va^2;

%Define vectors wbe_b and V_b
wbe_b = [x4; x5; x6]; % Angular velocity vector in body frame
V_b = [x1; x2; x3]; % Velocity vector in body frame

%3. AERODYNAMIC FORCE COEFFICIENTS
%Calculate the CL_wb
if alpha < alpha_switch
    CL_wb = n * (alpha - alpha_LO); % Lift coefficient for wing-body
else
    CL_wb = a3 * alpha^3 + a2 * alpha^2 + a1 * alpha + a0; % Nonlinear lift coefficient
end

%Calculate CL_t
epsilon = depsa * (alpha - alpha_LO); % Downwash angle
alpha_t = alpha - epsilon + u2 + 1.3 * x5 * lt / Va; % Tail angle of attack
CL_t = 3.1 * (St / S) * alpha_t; % Lift coefficient for tail

%Total lift force
CL = CL_wb + CL_t; % Total lift coefficient

%Total drag force (neglecting tail)
CD = 0.13 + 0.07 * (5.5 * alpha + 0.654)^2; % Drag coefficient

%Calculate Sideforce
CY = -1.6 * beta + 0.24 * u3; % Side force coefficient

%4. DIMENSIONAL AERODYNAMIC FORCES
%Calculate the actual dimensional forces in the F_s (stability axis)
FA_s = [-CD * Q * S;
         CY * Q * S;
        -CL * Q * S];

%Rotate these forces to F_b (body axis)
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];

FA_b = C_bs * FA_s; % Aerodynamic forces in body frame

% 5. AERODYNAMIC MOMENT COEFFICIENT ABOUT AC
eta11 = -1.4 * beta;
eta21 = -0.59 - (3.1 * (St * lt) / (S * cbar)) * (alpha - epsilon);
eta31 = (1 - alpha * (180 / (15 * pi))) * beta;

eta =  [eta11;
        eta21;
        eta31];

dCMdx = (cbar / Va) * [-11 0 5;
                    0 (-4.03 * (St * lt^2) / (S * cbar^2)) 0;
                    1.7 0 -11.5];

dCMdu = [-0.6 0 0.22;
            0 (-3.1 * (St * lt) / (S * cbar)) 0;
            0 0 -0.63];

%Now calc CM = [C1;Cm;Cn] about Aerodynamic centre in Fb
CMac_b = eta + dCMdx * wbe_b + dCMdu * [u1; u2; u3];

% 6. AERODYNAMIC MOMENT ABOUT AC
%Normalize to an aerodynamic moment
MAac_b = CMac_b * Q * S * cbar;

% 7. AERODYNAMIC MOMENT ABOUT CG
%Transfer moment to cg
rcg_b = [Xcg; Ycg; Zcg];
rac_b = [Xac; Yac; Zac];
MAcg = MAac_b + cross(FA_b, rcg_b - rac_b);

%ENGINE FORCE AND MOMENT
%Thrust
F1 = u4 * m * g;
F2 = u5 * m * g;

%Assuming engine thrust is aligned with Fb
FE1_b = [F1; 0; 0];
FE2_b = [F2; 0; 0];

FE_b = FE1_b + FE2_b; % Total engine force in body frame

%Moment due to engines offset from CoG
mew1 = [Xcg - Xapt1;
        Yapt1 - Ycg;
        Zcg - Zapt1];

mew2 = [Xcg - Xapt2;
        Yapt2 - Ycg;
        Zcg - Zapt2];

MEcg1_b = cross(mew1, FE1_b);
MEcg2_b = cross(mew2, FE2_b);

MEcg_b = MEcg1_b + MEcg2_b; % Total engine moment in body frame

% 9. GRAVITY EFFECTS
%Calculate gravitational forces in body frame. Causes no moment about CG
g_b = [-g * sin(x8);
        g * cos(x8) * sin(x7);
        g * cos(x8) * sin(x7)];

Fg_b = m * g_b; % Gravitational force in body frame

% 10. STATE DERIVATIVES

%Inertia Matrix
Ib = m * [40.07 0 -2.0923;
        0 64 0;
        -2.0923 0 99.92];

invIb = (1 / m) * [0.0249836 0 0.0005231;
                0 0.0156 0;
                0.000523 0 0.01];

%Form F_b (all forces in Fb) and calc udot, vdot, wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1 / m) * F_b - cross(wbe_b, V_b); % Translational accelerations

%Form Mcg_b (All moments about CoG in Fb) and calc pdot, qdot, rdot
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb * (Mcg_b - cross(wbe_b, Ib * wbe_b)); % Rotational accelerations

%Calculate Phidot, thetadot, and psidot
H_phi = [1 sin(x7) * tan(x8) cos(x7) * tan(x8);
        0 cos(x7) -sin(x7);
        0 sin(x7) / cos(x8) cos(x7) / cos(x8)];

x7to9dot = H_phi * wbe_b; % Angular rates

%Place in first order form
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot]; % State derivatives