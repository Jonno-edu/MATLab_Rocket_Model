function H = ComputeMeasurementJacobian(mag_pred)
    % Computes the measurement Jacobian matrix
    % mag_pred: predicted magnetometer reading in body frame (3x1)
    
    % H = [SkewSymmetric(mag_pred), zeros(3,3)]
    H = [SkewSymmetric(mag_pred), zeros(3,3)];
end
