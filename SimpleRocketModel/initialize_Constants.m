% Initialize constants for RCAM simulation
clear
clc
close all

%% Define Constants
x0 = [0;
      0;
      0;
      (90)*pi/180;
      0;
      0];

u = [100;
    (0.1)*pi/180];

burn_time = 10;

  
TF = 30;

%% Run the model
mdl = 'RocketSimulation.slx';

%% Plot the results
out = sim(mdl);
t = out.simX.Time;

% Control Inputs
u1 = out.simU.Data(:,1);  % Thrust (N)
u2 = out.simU.Data(:,2);  % Nozzle pitch angle (rad)
theta_error = out.simU.Data(:,3);  % Ref body angle (rad)

% Instantaneous States
u     = out.simX.Data(:,1);  % Body x-axis velocity (m/s)
v     = out.simX.Data(:,2);  % Body y-axis velocity (m/s)
q     = out.simX.Data(:,3);  % Pitch rate (rad/s)
theta = out.simX.Data(:,4);  % Pitch angle (rad)


% Integrated States

% the integrated positions should be in columns 5 and 6.
xpos  = out.simX.Data(:,5);  % Integrated x-position (m)
ypos = out.simX.Data(:,6);  % Integrated y-position (m)

%% Plot Control Inputs
figure
subplot(2,1,1)
plot(t, u1, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Control Input u_1: Thrust')
grid on
legend('u_1')

subplot(2,1,2)
plot(t, u2, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Nozzle Pitch (rad)')
title('Control Input u_2: Nozzle Pitch')
grid on
legend('u_2')

%% Plot Instantaneous States
figure
% u: Body x-axis velocity
subplot(3, 2, 1)
plot(t, u, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('u (m/s)')
title('State x_1: Body x-axis Velocity')
grid on
legend('u')

% v: Body y-axis velocity
subplot(3, 2, 2)
plot(t, v, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('v (m/s)')
title('State x_2: Body y-axis Velocity')
grid on
legend('v')

% q: Pitch rate
subplot(3, 2, 3)
plot(t, q, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('q (rad/s)')
title('State x_3: Pitch Rate')
grid on
legend('q')

% theta: Pitch angle
subplot(3, 2, 4)
plot(t, theta, 'LineWidth', 1.5)
hold on
plot(t, theta_error, 'LineWidth', 1.5)
hold off
xlabel('Time (s)')
ylabel('\theta (rad)')
title('State x_4: Pitch Angle')
grid on
legend('theta')

% u: EARTH x-axis pos
subplot(3, 2, 5)
plot(t, xpos, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('u (m/s)')
title('State x_1: Earth x-axis pos')
grid on
legend('u')

% v: EARTH y-axis pos
subplot(3, 2, 6)
plot(t, ypos, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('v (m/s)')
title('State x_2: Earth y-axis pos')
grid on
legend('v')


%% Plot Trajectory
figure

% trajectory
subplot(1, 1, 1)
plot(xpos, ypos, 'LineWidth', 1.5)
xlabel('X position')
ylabel('Y position')
title('Trajectory')
grid on
legend('trajectory')
axis equal

figure
% theta: Pitch angle
subplot(1, 1, 1)
plot(t, theta, 'LineWidth', 1.5)
hold on
plot(t, theta_error, 'LineWidth', 1.5)
hold off
xlabel('Time (s)')
ylabel('\theta (rad)')
title('State x_4: Pitch Angle')
grid on
legend('theta')
