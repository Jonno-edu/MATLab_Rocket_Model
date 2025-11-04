function v_body = RotateVectorByQuaternion(v_inertial, q)
    % Rotates a vector from inertial frame to body frame using quaternion
    % v_inertial: vector in inertial frame (3x1)
    % q: quaternion [w; x; y; z]
    
    % Convert quaternion to rotation matrix
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    R = [1-2*(y^2+z^2),  2*(x*y-w*z),    2*(x*z+w*y);
         2*(x*y+w*z),    1-2*(x^2+z^2),  2*(y*z-w*x);
         2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x^2+y^2)];
    
    % Rotate vector
    v_body = R' * v_inertial; % R' gives inertial to body
end
