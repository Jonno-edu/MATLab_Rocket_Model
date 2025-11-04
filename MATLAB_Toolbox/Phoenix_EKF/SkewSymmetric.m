function S = SkewSymmetric(v)
    % Creates a skew-symmetric matrix from a 3D vector
    % v = [vx; vy; vz]
    
    S = [  0,    -v(3),   v(2);
          v(3),    0,    -v(1);
         -v(2),  v(1),     0  ];
end
