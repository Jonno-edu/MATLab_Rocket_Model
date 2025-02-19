% Initialize constants for Rocket simulation
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

u = [4;
    (0.5)*pi/180];

burn_time = 10;

  
TF = 30;

%% Run the model
mdl = 'RocketSimulation.slx';
% Updated sim call to include stop time for finite outputs:
out = sim(mdl, 'StopTime', num2str(TF));

%% Plot the results
t = out.simX.Time;

% Control Inputs
u1 = out.simU.Data(:,1);  % Thrust (N)
u2 = out.simU.Data(:,2);  % Nozzle Pitch (rad)
theta_error = out.simU.Data(:,3);  % Ref body angle (rad)
q_error = out.simU.Data(:,4);  % Ref body angle rate (rad/s)

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
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Control Input u_1: Thrust')
grid on
legend('u_1', 'Motor Cutoff')

subplot(2,1,2)
plot(t, u2, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('Nozzle Pitch (rad)')
title('Control Input u_2: Nozzle Pitch')
grid on
legend('u_2', 'Motor Cutoff')

%% Plot Instantaneous States
figure
% u: Body x-axis velocity
subplot(3, 2, 1)
plot(t, u, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('u (m/s)')
title('State x_1: Body x-axis Velocity')
grid on
legend('u', 'Motor Cutoff')

% v: Body y-axis velocity
subplot(3, 2, 2)
plot(t, v, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('v (m/s)')
title('State x_2: Body y-axis Velocity')
grid on
legend('v', 'Motor Cutoff')

% q: Pitch rate
subplot(3, 2, 3)
plot(t, q, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('q (rad/s)')
title('State x_3: Pitch Rate')
grid on
legend('q', 'Motor Cutoff')

% theta: Pitch angle
subplot(3, 2, 4)
plot(t, theta, 'LineWidth', 1.5)
hold on
plot(t, theta_error, 'LineWidth', 1.5)
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('\theta (rad)')
title('State x_4: Pitch Angle')
grid on
legend('theta', 'theta_{error}', 'Motor Cutoff')

% u: EARTH x-axis pos
subplot(3, 2, 5)
plot(t, xpos, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('u (m/s)')
title('State x_1: Earth x-axis pos')
grid on
legend('u', 'Motor Cutoff')

% v: EARTH y-axis pos
subplot(3, 2, 6)
plot(t, ypos, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('v (m/s)')
title('State x_2: Earth y-axis pos')
grid on
legend('v', 'Motor Cutoff')


%% Plot Trajectory
figure

% trajectory
subplot(1, 1, 1)
plot(xpos, ypos, 'LineWidth', 1.5)
hold on
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('X position')
ylabel('Y position')
title('Trajectory')
grid on
legend('trajectory', 'Motor Cutoff')
axis equal

figure

subplot(1, 2, 1)
h1 = plot(t, theta, 'LineWidth', 1.5);
hold on
h2 = plot(t, theta_error, 'LineWidth', 1.5);
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('\theta (rad)')
title('State x_4: Pitch Angle')
grid on
legend([h1, h2], '\theta_{measured}', '\theta_{setpoint}', 'Motor Cutoff')

subplot(1, 2, 2)
h3 = plot(t, q, 'LineWidth', 1.5);
hold on
h4 = plot(t, q_error, 'LineWidth', 1.5);
xline(burn_time, '--r', 'LineWidth', 1.5); % Motor cutoff point
hold off
xlabel('Time (s)')
ylabel('q (rad/s)')
title('State x_5: Pitch Angle Rate') 
grid on
legend([h3, h4], 'q_{measured}', 'q_{setpoint}', 'Motor Cutoff')

