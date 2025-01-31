%Initialize constants for RCAM simulation
clear
clc
close all

%% Define Constants
x0 = [0;
    0;
    0;
    90 * pi/180];

u = [200;
    0];


TF = 3*60;


%% Run the model
sim('RocketSimulation.slx')