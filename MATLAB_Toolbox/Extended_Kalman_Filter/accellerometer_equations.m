clear; clc; close all;

g = 9.81

theta = deg2rad(30)

real_ax = 5
real_az = 10

% Our raw accelerometer measurements
disp("Raw specific Force Measurements\n")
fx = -real_ax - g*sin(theta)
fz = -real_az + g*cos(theta)

% Our caluclated Body accelerations
disp("Body Accelleration Measurements\n")
ax = -fx - g*sin(theta)
az = -fz + g*cos(theta)

% Convert measured AB to ND frame
disp("ND Accelleration Measurements\n")
aN_B = ax*cos(theta) + az*sin(theta)
aD_B = az*cos(theta) - ax*sin(theta)

% Convert raw accel measurements to Accelleration Inertial
aN_Fs = (-fx - g*sin(theta))*cos(theta) + (-fz + g*cos(theta))*sin(theta)
aD_Fs = (-fz + g*cos(theta))*cos(theta) - (-fx - g*sin(theta))*sin(theta)

aN_Fs = (-fx)*cos(theta) + (-fz)*sin(theta)
aD_Fs = (-fz)*cos(theta) - (-fx)*sin(theta) + g

