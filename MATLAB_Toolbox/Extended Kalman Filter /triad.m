function [phi_meas, theta_meas, psi_meas] = triad(g_I, mag_I, g_b, mag_b)

% Goal: Determin the DCM that satisfies:
% g_B = DCM_IB * g_I
% mag_B = DCM_IB * mag_I

% 1. Construct a triad of orthorormal vectors S1, S2, S3 in the body axes fro
% mthe measured g_B and mag_B

s_1 = g_b/norm(g_b);
s_2 = cross(s_1, mag_b)/norm(cross(s_1, mag_b));
s_3 = cross(s_1, s_2);


% 2. Construct a triad of orthonormal vectors r1, r2, r3 in the inertial
% axes from the reference vectors g_I and mag_I

r_1 = g_I/norm(g_I);
r_2 = cross(r_1, mag_I)/norm(cross(r_1, mag_I));
r_3 = cross(r_1, r_2);

% 3. Calculate the DCM that transforms the inertial TRIAD r1, r2, r3 to the
% body TRIAD s1, s2, s3
% [s1 s2 s3] = DCM_IB[r1 r2 r3]
S = [s_1 s_2 s_3];
R = [r_1 r_2 r_3];

DCM_IB = S*R';

DCM23 = DCM_IB(2, 3);
DCM33 = DCM_IB(3, 3);

DCM13 = DCM_IB(1, 3);

DCM12 = DCM_IB(1, 2);
DCM11 = DCM_IB(1, 1);

phi_meas = atan2(DCM23,DCM33);
theta_meas = -asin(DCM13);
psi_meas = atan2(DCM12,DCM11);

end