function sys = rocket_with_actuator_states( ...
    m, I, V, rho, S, L, T, L_arm, CN_alpha, Cm_alpha, Cm_q, ...
    actuator_wn, actuator_zeta )
%ROCKET_WITH_ACTUATOR_SIMULINK_ORDER Rocket pitch + 2nd-order actuator in Simulink state order

% --- Derived (Dimensional) Stability Derivatives
q_dyn   = 0.5 * rho * V^2;
M_delta = T * L_arm / I;
M_q     = q_dyn * S * L^2 / (2 * V * I) * Cm_q;
% If you want direct δ–>q coupling (M_delta)
% Plant does not depend on actuator velocity, only deflection

% --- Actuator second-order system (δ, δ̇)
wn   = actuator_wn;
zeta = actuator_zeta;

% --- Build the A, B in Simulink ordering: [q, actuator_input, actuator_defl, actuator_rate]
A = zeros(4,4);
B = zeros(4,1);

% 1. Pitch rate dynamics: dot(q) = plant terms + actuator deflection
A(1,1) = M_q;        % q coupling (pitch damping)
A(1,3) = M_delta;    % actuator deflection to q

% 2. Actuator "input" dynamics (x2): pure delay/hold, i.e., dot(x2) = 0, u enters here
A(2,:) = 0;          % dot(x2) = 0
B(2,1) = 1;          % input (nozzle_angle_cmd) enters here

% 3. Actuator deflection (x3): dot(x3) = x4
A(3,4) = 1;          % deflection integrates actuator rate

% 4. Actuator rate (x4): dot(x4) = -wn^2*x3 - 2*zeta*wn*x4 + wn^2*x2
A(4,2) = wn^2;                        % actuator input to rate
A(4,3) = -wn^2;                       % actuator position feedback
A(4,4) = -2*zeta*wn;                  % actuator velocity feedback

% Output: pitch rate (q)
C = [1 0 0 0];
D = 0;

Ts = 0.0001; % match your Simulink discrete time, or Ts = 0 for continuous
sys = ss(A,B,C,D,Ts);

end
