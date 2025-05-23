clear
clc
close all

%% Plant model (DC Motor)
%Define Constants

R = 0.05;
KV = 0.09854;
KT = 0.09854;
Rm = 1.5398;
La = 0.0015581;
c = 0.00039719;
Jm = 0.00137;

%State Space Representation
A = [0 1 0;
     0 -c/Jm KT/Jm;
     0 -KV/La -(R + Rm)/La];

B = [0 0;
     0 -1/Jm;
     1/La 0];

C = eye(3);

D = zeros(3, 2);

open_loop_poles = eig(A)

%% Design Full State Feedback Controller
%Verify that the system is controllable
Pc = ctrb(A,B(:,1))
rank(Pc)

desired_closed_loop_poles = [-100; -110; -120];

%Compute full state feedback gain
K = place(A, B(:,1), desired_closed_loop_poles)

%% Simulate System
t_final = 2;
x0 = [75*pi/180;
      2*pi;
      -1];

simOut = sim('practicalFSFB_model');
logsout = simOut.logsout;

%From model with full state feedback

sim_X = logsout.getElement('x').Values;

t = sim_X.Time;
x1 = sim_X.Data(:,1);
x2 = sim_X.Data(:,2);
x3 = sim_X.Data(:,3);

sim_Va = logsout.getElement('Va').Values;
Va = sim_Va.Data(:,1);

%Create Plots
figure
subplot(3,1,1)
plot(t, x1, 'LineWidth',2)
ylabel('x_1(t)')

subplot(3,1,2)
plot(t, x2, 'LineWidth',2)
ylabel('x_2(t)')

subplot(3,1,3)
plot(t, x3, 'LineWidth',2)
ylabel('x_3(t)')

figure
plot(t, Va, 'LineWidth', 2)
ylabel('V_a(t)')
grid on
title('Control Signal, V_a')