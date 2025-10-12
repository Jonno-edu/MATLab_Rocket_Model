%% Transfer Function: Pitch Angle vs Nozzle Deflection Angle
% For a point mass with fixed inertia and thrust vector control
% Author: TVC Dynamics Script
% Date: October 2025

clear; clc;

%% System Parameters
% Define your rocket parameters here
T = 0.75*9.81;           % Thrust [N]
l = 0.52;            % Moment arm (distance from gimbal to CG) [m]
Iyy = 0.1896;           % Pitch moment of inertia [kg*m^2]

%% Transfer Function Derivation
% The pitch dynamics are governed by:
% Iyy * d²θ/dt² = M_pitch
% Where M_pitch = -T * sin(δ) * l ≈ -T * δ * l (for small angles)
%
% Taking Laplace transform with zero initial conditions:
% Iyy * s² * Θ(s) = -T * l * δ(s)
%
% Transfer function: Θ(s)/δ(s) = -T*l / (Iyy * s²)

%% Create Transfer Function
% Numerator: -T * l
num = -T * l;

% Denominator: Iyy * s²
den = [Iyy 0 0];  % Represents Iyy*s² + 0*s + 0

% Create transfer function object
s = tf('s');
G_pitch = num / (Iyy * s^2);

% Display transfer function
disp('Transfer Function: Pitch Angle (θ) / Nozzle Angle (δ)')
disp(G_pitch)

