function F = ComputeStateTransitionMatrix(omega, dt)
    % Computes the discrete state transition matrix for the error state
    % omega: bias-corrected angular velocity (3x1)
    % dt: time step
    
    % Skew-symmetric matrix of omega
    omega_skew = SkewSymmetric(omega);
    
    % Continuous-time state transition matrix
    F_continuous = [-omega_skew, -eye(3);
                    zeros(3,3),  zeros(3,3)];
    
    % First-order discretization
    F = eye(6) + F_continuous * dt;
end
