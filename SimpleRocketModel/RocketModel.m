function [XDOT] = RocketModel(X, U)
    % State Variables
    u = X(1);       % Velocity along body x-axis 
    v = X(2);       % Velocity along body y-axis
    q = X(3);       % Pitch rate
    theta = X(4);   % Pitch angle
    
    % Extract control inputs
    thrust = U(1);          % Thrust magnitude (N)
    nozzlePitch = U(2);     % Nozzle pitch angle (rad)
    
    % Constants
    m = 3.9;      % Rocket total mass (kg)
    l = 1;        % Distance from CoM to nozzle
    g = 9.81;     % Gravitational acceleration (m/s²)
    Ib = 1037524.81888/(1000*1000);  % Moment of inertia
    invIb = 1 / Ib;
    
    % Thrust force in body frame
    gamma = nozzlePitch;
    Fp_b = thrust * [cos(gamma); sin(gamma)];
    
    % Gravity force in body frame (depends on actual theta)
    g_b = [-g * sin(theta); 
           -g * cos(theta)];
    Fg_b = m * g_b;
    
    % Net force in body frame
    F_b = Fp_b + Fg_b;
    
    % Compute accelerations with Coriolis terms
    udot = F_b(1)/m + q*v;
    vdot = F_b(2)/m - q*u;
    
    % Moment about CoM and angular acceleration
    Mcg_b = -l * Fp_b(2);
    qdot = invIb * Mcg_b;
    
    % Theta dot is the pitch rate
    thetadot = q;
    
    % Navigation equations (body to global velocity)
    vx_v = u * cos(theta) - v * sin(theta);
    vy_v = u * sin(theta) + v * cos(theta);
    
    % State derivatives
    XDOT = [udot; vdot; qdot; thetadot; vx_v; vy_v];
end