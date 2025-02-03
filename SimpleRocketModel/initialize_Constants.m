%Initialize constants for RCAM simulation
clear
clc
close all

%% Define Constants
x0 = [0;
    0;
    0;
    90*pi/180];

u = [100;
      0];
  

TF = 10;


%% Run the model
mdl = 'RocketSimulation.slx';
sim(mdl)

%% Plot the results
out = sim(mdl);
t = out.simX.Time;

% Control Inputs
u1 = out.simU.Data(:,1);  % Thrust (N)
u2 = out.simU.Data(:,2);  % Nozzle pitch angle (rad)

% Instantaneous States
u    = out.simX.Data(:,1);  % Body x-axis velocity (m/s)
v    = out.simX.Data(:,2);  % Body y-axis velocity (m/s)
q    = out.simX.Data(:,3);  % Pitch rate (rad/s)
theta= out.simX.Data(:,4);  % Pitch angle (rad)

% Integrated States (e.g., positions)
% NOTE: Verify that your simulation outputs integrated states in these columns.
xpos = out.simX.Data(:,1);  % Integrated x-position (m)
xypos= out.simX.Data(:,2);  % Integrated y-position (m)

figure

%% Plot u1 (Thrust)
subplot(2,1,1)
plot(t, u1, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Control Input u_1: Thrust')
grid on
legend('u_1')

%% Plot u2 (Nozzle Pitch)
subplot(2,1,2)
plot(t, u2, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Nozzle Pitch (rad)')
title('Control Input u_2: Nozzle Pitch')
grid on
legend('u_2')



%% Plot Instantaneous States
figure
% u: Body x-axis velocity
subplot(2, 2, 1)
plot(t, u, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('u (m/s)')
title('State x_1: Body x-axis Velocity')
grid on
legend('u')

% v: Body y-axis velocity
subplot(2, 2, 2)
plot(t, v, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('v (m/s)')
title('State x_2: Body y-axis Velocity')
grid on
legend('v')

% q: Pitch rate
subplot(2, 2, 3)
plot(t, q, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('q (rad/s)')
title('State x_3: Pitch Rate')
grid on
legend('q')

% theta: Pitch angle
subplot(2, 2, 4)
plot(t, theta, 'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\theta (rad)')
title('State x_4: Pitch Angle')
grid on
legend('theta')
