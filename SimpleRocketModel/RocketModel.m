function [XDOT] = RocketModel(X, U)
    % Visualization of Both Frames:
    % ---------------------------------
    %
    %   Earth Frame (Inertial)
    %         ^ Y (Up, Altitude)
    %         |  
    %         |  
    %         o-------> X (East)
    %
    %   Body Frame (rotates with rocket)
    %          ^
    %      Y_b |   
    %          |-----> X_b  (Rocket forward)
    
    % State Variables
    u = X(1);       % Velocity along body x-axis 
    v = X(2);       % Velocity along body y-axis
    q = X(3);       % Pitch rate
    theta = X(4);   % Pitch angle
    
    % Extract control inputs
    thrust = U(1);          % Thrust magnitude (N)
    nozzlePitch = U(2);     % Nozzle pitch angle (rad)
    
    % CONSTANTS
    m = 63/1000;      % Rocket total mass (kg)
    l_p = 1;    % Distance from aerodynamic centre to centre of mass
    l   = 24.8/100;  % Distance from centre of mass to nozzle
    
    % Environmental constants
    rho = 1.225; % Air density (kg/m^3)
    g = 9.81;    % Gravitational acceleration (m/s^2)
    
    % CONTROL LIMITS/SATURATION
    thrust_min = 0;         % min thrust (N)
    thrust_max = 500;       % max thrust (N)
    nozzle_min = -10 * pi / 180; % Min nozzle pitch (rad)
    nozzle_max = 10 * pi / 180;  % Max nozzle pitch (rad)
    

    
    % Inertia Matrix for 2D (simplified)
    Ib = 0.000995;  % Example inertia value
    invIb = 1 / Ib;
    
    % THRUST FORCE AND MOMENT
    % Use nozzle pitch angle for thrust direction
    gamma = nozzlePitch;
    Fp = thrust;  % Thrust magnitude (N)
    Fp_b = Fp * [cos(gamma); sin(gamma)]; 
    
    % GRAVITY EFFECTS in Body Frame
    % Convert gravity to body frame using pitch angle theta
    g_b = [-g * sin(theta); 
           -g * cos(theta)];
    Fg_b = m * g_b;  % = m * [-g*sin(theta); -g*cos(theta)]
    
    % Net Force in the Body Frame
    F_b = Fp_b + Fg_b;
    
    % Compute accelerations (Newton's Second Law)
    udot = F_b(1) / m;
    vdot = F_b(2) / m;
    
    % Angular Dynamics (Moment about centre of gravity)
    % Here we assume moment arm is l and only the y-component of thrust causes moment.
    Mcg_b = -l * Fp_b(2);
    qdot = invIb * Mcg_b;
    
    % Theta dot is the pitch rate
    thetadot = q;

    %Navigation Equations
    vx_v = u * cos(theta) - v * sin(theta);
    vy_v = u * sin(theta) + v * cos(theta);
 
    % Return state derivatives in first-order form
    XDOT = [udot;
            vdot;
            qdot;
            thetadot;
            vx_v;
            vy_v];
end
