function q = SmallAngleToQuaternion(theta)
    % Converts a small rotation vector to a quaternion
    % theta: rotation vector (3x1) in radians
    
    angle = norm(theta);
    
    if angle > 1e-9
        axis = theta / angle;
        q = [cos(angle/2);
             axis(1) * sin(angle/2);
             axis(2) * sin(angle/2);
             axis(3) * sin(angle/2)];
    else
        % For very small angles, use first-order approximation
        q = [1; theta(1)/2; theta(2)/2; theta(3)/2];
        q = q / norm(q);
    end
end
