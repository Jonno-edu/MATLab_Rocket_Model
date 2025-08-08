clc; clear; close all;

t = 0:0.01:10;
s = tf('s');
P_pitch = (1.151*s + 0.1774)/(s^3 + 0.739*s^2 + 0.921*s)
step(0.2*P_pitch,t); %scaling the step response by 0.2. Step input is 0.2 rad 11 deg
axis([0 10 0 0.8]);
ylabel('pitch angle (rad)');
title('Open-loop Step Response');
grid

K = 1;
margin(K*P_pitch), grid
figure;

K = 10;
margin(K*P_pitch), grid
figure;
hold off
sys_cl = feedback(K*P_pitch,1);
step(0.2*sys_cl), grid
title('Closed-loop Step Response with K = 10')

% We are required to have overshoot of < 10% 
% Damping ratio approx > 0.59
% Therefore phase margin of > 59 degrees
% Since our current phase margin with K = 10 is appox: 10.4 degrees
% We need an additional 50 degrees of phase lead to account for the fact
% that our gain crossover will increase to a point where the system has
% more phase lag.
% We arbitrarily add 5 degrees for a total bump of 5 + 50 = 55 degrees.
% We can use this number to solve alpha in: 
% alpha = (1 - sin(55))/(1 +sin(55)) = 0.10
% From above, we calc we need alpha less than approx 0.10
% The following relationship used to determine the ammount of magnitude
% increase that will be supplied by the lead compensator at the location of
% the max bump in phase:
% 20log(1/sqrt(alpha)) = 20log(1/(sqrt(0.1)) = 10dB
% Examining the bode plot, the magnitude of uncompensated system = -10dB at
% 6.1 rad/s. Therefor, the addition of our lead compensator will move the
% gain crossover freq from 3.49rad/s to 6.1rad/s.
% Using this info, we can calculate the value T from the following in order
% to centre the maximum bump in phase at the new gain crossover frequency
% in order to maximise the system's resulting phase margin:
% w_m = 1/(T*sqrt(alpha)) => T = 1/(6.1*sqrt(0.1)) = 0.52
% With the values from above, we will attampt our lead compensator:


K = 10;
alpha = 0.1;
T = 0.52;
C_lead = K*(T*s + 1)/(alpha * T * s + 1);

margin(C_lead*P_pitch), grid

sys_cl_1 = feedback(C_lead*P_pitch, 1);
step(0.2*sys_cl), grid
stepinfo(0.2*sys_cl)


% Iterating on the design process: We come up with new alpha and T

K = 10;
alpha = 0.04;
T = 0.55;
C_lead = K*(T*s + 1) / (alpha*T*s + 1);
sys_cl_2 = feedback(C_lead*P_pitch,1);
step(0.2*sys_cl), grid
stepinfo(0.2*sys_cl)
title('Closed-loop Step Response with K = 10, \alpha = 0.04, and T = 0.55')

