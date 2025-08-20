
function C_pqr = pqr_to_euler_rate(phi_prev, theta_prev)
sphi = sin(phi_prev); ttheta = tan(theta_prev);
cphi = cos(phi_prev); 
sectheta = sec(theta_prev); 

C_pqr = [ ...
    1,  sphi*ttheta,    cphi*ttheta; ...
    0,  cphi,           -sphi; ...
    0,  sphi*sectheta,  cphi*sectheta];

end

