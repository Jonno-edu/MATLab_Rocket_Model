syms s d1 d2 d3 d4 d5
syms alpha theta thetadot nozzleAngle A_n V

% Example "Laplace domain" equations
eq1 = s*theta - s*alpha == d1*alpha + d5*nozzleAngle
eq2 = s^2*theta == d2*nozzleAngle + d3*alpha + s*d4*theta
eq3 = thetadot == s * theta
eq4 = A_n == V * thetadot



% Solve for alpha, theta in terms of nozzleAngle
sol = solve([eq1, eq2, eq3, eq4], [alpha, theta, thetadot, A_n])


% Transfer function alpha / nozzleAngle
TF1 = simplify(sol.alpha / nozzleAngle)

% Transfer function thetadot/ nozzleAngle
TF2 = simplify(sol.thetadot / nozzleAngle)

TF3 = simplify(sol.A_n / nozzleAngle)