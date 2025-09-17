% Inputs in degrees: [phi roll, theta pitch, psi yaw]
phi=45
theta=45 
psi=0
ang = deg2rad([psi, theta, phi]);  % [yaw pitch roll] for ZYX

% ZYX nav->body DCM
cz=cos(ang(1)); sz=sin(ang(1));
cy=cos(ang(2)); sy=sin(ang(2));
cx=cos(ang(3)); sx=sin(ang(3));
Rz=[cz -sz 0; sz cz 0; 0 0 1];
Ry=[cy 0 sy; 0 1 0; -sy 0 cy];
Rx=[1 0 0; 0 cx -sx; 0 sx cx];
R_bn = Rx*Ry*Rz

g_n = [0;0;9.81];
a_b = R_bn * g_n  % expected IMU reading at rest
