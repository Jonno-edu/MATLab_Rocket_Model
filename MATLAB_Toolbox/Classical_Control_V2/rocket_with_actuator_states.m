% Parameters (update as needed)
omega_n = 62;          % natural frequency (rad/s)
zeta = 0.505;            % damping ratio

T = 27607;        % Newtons (example)
l_CG = 5.55;       % meters (example)
I_y = 21553;      % kg*m^2 (example)

% Actuator transfer function
num_act = [omega_n^2];
den_act = [1 2*zeta*omega_n omega_n^2];

% Plant transfer function (pitch rate output)
k_plant = T * l_CG / I_y;
num_plant = [k_plant];
den_plant = [1 0];    % Integrator

% Series connection (full open loop system: command -> pitch rate)
num_total = conv(num_act, num_plant);
den_total = conv(den_act, den_plant);

sys_total = tf(num_total, den_total)

