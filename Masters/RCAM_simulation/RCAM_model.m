function [XDOT] = RCAM_model(X, U)

%STATE AND CONTROL VECTOR
%Extract state vector

x1 = X(1); %u
x2 = X(2); %v
x3 = X(3); %w
x4 = X(4); %p
x5 = X(5); %q
x6 = X(6); %r
x7 = X(7); %phi
x8 = X(8); %theta
x9 = X(9); %psi

u1 = U(1); %d_A (aileron)
u2 = U(2); %d_T (stabilizer)
u3 = U(3); %d_R (rudder)
u4 = U(4); %d_th1 (throttle 1)
u5 = U(5); %d_th2 (throttle 2)

%CONSTANTS
m = 120000;                 %Aircraft total mass (kg)

cbar = 6.6;
lt = 24.8;
S = 260;
St = 64;

Xcg = 0.23*cbar;
Ycg = 0;
Zcg = 0.1*cbar;

Xac = 0.12*cbar;
Yac = 0;
Zac = 0;

%Engine constants
Xapt1 = 0;
Yapt1 = -7.94;
Zapt1 = -1.9;

Xapt2 = 0;
Yapt2 = 7.94;
Zapt2 = -1.9;

rho = 1.225;
g = 9.81;
depsa = 0.25;
alpha_LO = -11.5*pi/180;
n = 5.5;
a3 = -768.5;
a2 = 609.2;
a1 = -155.2;
a0 = 15.212;
alpha_switch = 14.5*(pi/180);

%CONTROL LIMITS/SATURATION

ulmin1 = -25*pi/180;
ulmax1 = 25*pi/180;

ulmin2 = -25*pi/180;
ulmax2 = 10*pi/180;

ulmin3 = -30*pi/180;
ulmax3 = 30*pi/180;

ulmin4 = 0.5*pi/180;
ulmax4 = 10*pi/180;

ulmin5 = 0.5*pi/180;
ulmax5 = 10*pi/180;

if(u1>ulmax1)
    u1 = ulmax1;
elseif(u1<ulmin1)
    u1 = ulmin1;
end


if(u2>ulmax2)
    u2 = ulmax2;
elseif(u2<ulmin2)
    u2 = ulmin2;
end


if(u3>ulmax3)
    u3 = ulmax3;
elseif(u3<ulmin3)
    u3 = ulmin3;
end


if(u4>ulmax4)
    u4 = ulmax4;
elseif(u4<ulmin4)
    u4 = ulmin4;
end


if(u5>ulmax5)
    u5 = ulmax5;
elseif(u5<ulmin5)
    u5 = ulmin5;
end


%2. INTERMEDIATE VARIABLES
%Calculate airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

%Calculate alpha and beta
alpha = atan2(x3, x1);
beta = asin(x2/Va);

%Calculate Dynamic pressure
Q = 0.5*rho*Va^2;

%Define vectors wbe_b and V_b
wbe_b = [x4; x5; x6];
V_b = [x1; x2; x3];

%3. AERODYNAMIC FORCE COEFFICIENTS
%Calculate the CL_wb

if alpha<alpha_switch
    CL_wb = n*(alpha - alpha_LO);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

%Calculate CL_t
epsilon = depsa*(alpha - alpha_LO);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t = 3.1*(St/S)*alpha_t;

%Total lift force
CL = CL_wb + CL_t;

%Total drag force (neglecting tail)
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

%Calculate Sideforce
CY = -1.6*beta + 0.24*u3;

%4. DIMENSIONAL AERODYNAMIC FORCES
%Calculate the actual dimensional forces in the F_s (stability axis)

FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];

%Rotate these forces to F_b (body axis)
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];

FA_b = C_bs*FA_s;


% 5. AERODYNAMIC MOMENT COEFFICIENT ABOUT AC
eta11 = -1.4*beta;
eta21 = -0.59 - (3.1*(St*lt)/(S*cbar))*(alpha-epsilon);
eta31 = (1 - alpha*(180/(15*pi)))*beta;

eta =  [eta11;
        eta21;
        eta31];

dCMdx = (cbar/Va)*[-11 0 5;
                    0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
                    1.7 0 -11.5];

dCMdu = [-0.6 0 0.22;
            0 (-3.1*(St*lt)/(S*cbar)) 0;
            0 0 -0.63];

%Now calc CM = [C1;Cm;Cn] about Aerodynamic centre in Fb
CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];
  

% 6. AERODYNAMIC MOMENT ABOUT AC
%Normalize to an aerodynamic moment
MAac_b = CMac_b*Q*S*cbar;

% 7. AERODYNAMIC MOMENT ABOUT CG
%Transfer moment to cg
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg = MAac_b + cross(FA_b, rcg_b - rac_b);

%ENGINE FORCE AND MOMENT
%Thrust
F1 = u4*m*g;
F2 = u5*m*g;

%Assuming engine thrust is alligned with Fb
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];

FE_b = FE1_b + FE2_b;

%Moment due to engines offset from  CoG
mew1 = [Xcg - Xapt1;
        Yapt1 - Ycg;
        Zcg - Zapt1];

mew2 = [Xcg - Xapt2;
        Yapt2 - Ycg;
        Zcg - Zapt2];

MEcg1_b = cross(mew1, FE1_b);
MEcg2_b = cross(mew2, FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;

% 9. GRAVITY EFFECTS
%Calculate gravitational forces in body frame. Causes no moment about CG
g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*sin(x7)];

Fg_b = m*g_b;

% 10. STATE DERIVATIVES

%Inertia Matrix
Ib = m*[40.07 0 -2.0923;
        0 64 0;
        -2.0923 0 99.92];

invIb = (1/m)*[0.0249836 0 0.0005231;
                0 0.0156 0;
                0.000523 0 0.01];


%Form F_b (all forces in Fb) and calc udot, vdot, wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b, V_b);

%Form Mcg_b (All moments about CoG in Fb) and calc pdot, qdot, rdot
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b,Ib*wbe_b));

%Calculate Phidot, thetadot, and psidot
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
        0 cos(x7) -sin(x7);
        0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9dot = H_phi*wbe_b;

%Place in first order form
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];