% FUNCTION: QuaternionIntegration.m
function q_new = QuaternionIntegration(q_old, omega, dt)
    % Propagates a quaternion using an angular velocity vector
    
    omega_norm = norm(omega);
    
    if omega_norm < 1e-9 % Avoid division by zero if no rotation
        q_new = q_old;
        return;
    end
    
    % Create the delta quaternion representing this rotation
    angle = omega_norm * dt;
    axis = omega / omega_norm;
    
    q_delta = [cos(angle / 2); 
               axis(1) * sin(angle / 2);
               axis(2) * sin(angle / 2);
               axis(3) * sin(angle / 2)];
           
    % Update the old quaternion by multiplying with the delta (Hamilton product)
    q_new = QuaternionMultiply(q_old, q_delta);
    
    % Re-normalize to ensure it remains a unit quaternion
    q_new = q_new / norm(q_new);
end
