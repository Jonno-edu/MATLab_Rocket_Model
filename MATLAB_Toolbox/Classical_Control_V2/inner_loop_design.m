omega_n = 62;        % [rad/s], actuator natural freq
zeta = 0.54;         % actuator damping ratio

sys = rocket_with_actuator_states( ...
    50, 30, 140, 1.225, 0.024, 1.45, 1200, 0.18, 2.3, -0.65, -21, omega_n, zeta );

A = sys.A
B = sys.B
C = sys.C
D = sys.D
