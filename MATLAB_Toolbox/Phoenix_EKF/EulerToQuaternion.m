% FUNCTION: EulerToQuaternion.m
function q = EulerToQuaternion(phi, theta, psi)
    % Converts Euler angles (in degrees) to a quaternion [w, x, y, z]
    % Using ZYX rotation sequence
    
    % Convert degrees to radians
    phi_rad = deg2rad(phi);
    theta_rad = deg2rad(theta);
    psi_rad = deg2rad(psi);
    
    cy = cos(psi_rad * 0.5);
    sy = sin(psi_rad * 0.5);
    cp = cos(theta_rad * 0.5);
    sp = sin(theta_rad * 0.5);
    cr = cos(phi_rad * 0.5);
    sr = sin(phi_rad * 0.5);

    q = zeros(4, 1);
    q(1) = cr * cp * cy + sr * sp * sy; % w
    q(2) = sr * cp * cy - cr * sp * sy; % x
    q(3) = cr * sp * cy + sr * cp * sy; % y
    q(4) = cr * cp * sy - sr * sp * cy; % z
end
