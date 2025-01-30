%Initialize constants for RCAM simulation
clear
clc
close all

%% Define Constants
x0 = [85;
    0;
    0;
    0;
    0;
    0;
    0;
    0.1;        %pitched up about 5.7 deg
    0];

u = [0;
    -0.1;
    0;
    0.08;
    0.08];


TF = 3*60;


%% Run the model
sim('RCAMSimulation.slx')