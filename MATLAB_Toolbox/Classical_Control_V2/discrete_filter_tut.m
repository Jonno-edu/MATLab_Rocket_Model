clear
close all
clc

s = tf('s')

%Sample time
T = 2;
%Crit Freq
W0 = 0.7;
%Quality fac
Q = 1;

G1 = (s^2 + W0^2)/(s^2 + W0/Q*s + W0^2)


G1t = c2d(G1, T, 'tustin')
G1t_w = c2d(G1, T, ['Method', 'Tustin', 'PrewarpFrequency', W0])


bode(G1, G1t, G1t_w)