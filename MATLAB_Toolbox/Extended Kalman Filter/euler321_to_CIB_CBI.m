
function [C_IB, C_BI] = euler321_to_CIB_CBI(phi, theta, psi)
cphi = cos(phi);  sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);  spsi = sin(psi);

C_IB = [ ...
    cpsi*cth,                      spsi*cth,                      -sth; ...
   cpsi*sphi*sth - spsi*cphi,    spsi*sphi*sth + cpsi*cphi,    sphi*cth; ...
   cpsi*cphi*sth + spsi*sphi,    spsi*cphi*sth + cpsi*sphi,     cphi*cth];

C_BI = C_IB.';

end




