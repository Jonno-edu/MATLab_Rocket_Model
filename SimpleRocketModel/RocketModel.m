function [XDOT] = RocketModel(X, U)
    
    % State Variables
    u = X(1);       % Velocity along body x-axis 
    v = X(2);       % Velocity along body y-axis
    q = X(3);       % Pitch rate
    theta = X(4);   % Pitch angle
    
    % Extract control inputs
    thrust = U(1);          % Thrust magnitude (N);
    nozzlePitch = U(2);     % Nozzle pitch angle (rad);
    
    % Constants
    rocket_length = 42/100;
    CG = 26/100;
    CP = 32/100;

    m = 71.3/1000;        % Rocket total mass (kg);
    l = rocket_length - CG;          % Distance from CoM to nozzle (m);
    g = 9.81;       % Gravitational acceleration (m/s²);
    Ib = 0.001;  % Moment of inertia (kg·m²);
    invIb = 1 / Ib;
    
    % Aerodynamic Parameters
    rho = 1.225;    % Air density (kg/m³);
    Cd = 0.75;      % Drag coefficient[1];
    A_ref = 0.00049;% Reference area (π*(0.1m/2)^2);
    Cl_alpha = 0.5*2*pi; % Lift curve slope (per radian);
    l_fins = rocket_length - CP;   % Fin moment arm (m);
    
    % Thrust force in body frame
    gamma = nozzlePitch;
    Fp_b = thrust * [cos(gamma); sin(gamma)];
    
    % Gravity force in body frame
    g_b = [-g * sin(theta); 
           -g * cos(theta)];
    Fg_b = m * g_b;
    
    % Aerodynamic forces[1][2]
    V = sqrt(u^2 + v^2);                % Airspeed magnitude;
    alpha = atan2(v, u);                % Angle of attack;
    q_dyn = 0.5 * rho * V^2;            % Dynamic pressure;
    
    % Drag force (opposes motion)
    if (V > 0)
        F_drag = q_dyn * A_ref * Cd * [-u/V; -v/V];
    else
        F_drag = [0; 0];
    end
        
    
    % Normal force (lift) due to angle of attack
    F_lift = [0; q_dyn * A_ref * Cl_alpha * alpha];
    
    % Total aerodynamic forces
    F_aero = F_drag + F_lift;
    
    % Net force in body frame
    F_b = Fp_b + Fg_b + F_aero;
    
    % Compute accelerations with Coriolis terms
    udot = F_b(1)/m + q*v;
    vdot = F_b(2)/m - q*u;
    
    % Aerodynamic moment from fins[4][5]
    M_aero = -l_fins * F_lift(2);  % Negative sign for stability;
    
    % Total moment (thrust vectoring + aerodynamics)
    Mcg_b = l * Fp_b(2) + M_aero;
    qdot = invIb * Mcg_b;
    
    % Theta dot is the pitch rate
    thetadot = q;
    
    % Navigation equations (body to global velocity)
    vx_v = u * cos(theta) - v * sin(theta);
    vy_v = u * sin(theta) + v * cos(theta);
    
    % State derivatives
    XDOT = [udot; vdot; qdot; thetadot; vx_v; vy_v];
end
