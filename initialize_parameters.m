%% Initial Params
theta0 = 90;

%% Nozzle Actuators
wn_act = 1; % rad/s (update for actuators)
z_act = 0.3;
maxdef_nozzle = (30) * pi/180; % rad
mindef_nozzle = -maxdef_nozzle;
rate_lim_nozzle = (1000) * pi/180; % rad

nozzle_moment_arm = 0.1;

max_thrust = 50; % N


