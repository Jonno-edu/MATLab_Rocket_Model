% FUNCTION: QuaternionInverse.m
function q_inv = QuaternionInverse(q)
    % Calculates the inverse of a unit quaternion.
    % For a unit quaternion, the inverse is its conjugate.
    % Assumes q is a column vector [w; x; y; z].
    
    q_inv = [q(1); -q(2); -q(3); -q(4)];
end
