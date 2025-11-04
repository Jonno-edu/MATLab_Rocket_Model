% FUNCTION: QuaternionToEuler.m
function [roll, pitch, yaw] = QuaternionToEuler(q)
    % Converts a quaternion [w; x; y; z] to Euler angles in degrees.
    % Assumes a ZYX rotation sequence.

    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        % Handle the case of gimbal lock (pitch is +/- 90 degrees)
        pitch = sign(sinp) * pi / 2;
    else
        pitch = asin(sinp);
    end

    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);

    % Convert angles from radians to degrees
    roll = rad2deg(roll);
    pitch = rad2deg(pitch);
    yaw = rad2deg(yaw);
end
