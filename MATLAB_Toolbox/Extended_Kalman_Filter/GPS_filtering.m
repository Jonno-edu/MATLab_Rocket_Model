
load_data

%% Filtering
v_NED_GPS = squeeze(Sim.gps_Ve.Data);
N = length(v_NED_GPS);


% LPF
fc = 0.5; % Hz
rc = 1/(2*pi*fc);
lp_a = dt/(rc + dt);

v_NED_lpf = zeros(3, N);

a_NED_GPS = zeros(3, N);
a_NED_lpf = zeros(3, N);


for k = 2:N
    v_NED_lpf(:, k) = lp_a * v_NED_GPS(:, k) + (1-lp_a)*v_NED_lpf(:, k - 1);
    a_NED_lpf(:, k) = (v_NED_lpf(:, k) - v_NED_lpf(:, k - 1))/dt;

end


figure;
plot(v_NED_lpf(1, :)); hold on;
plot(v_NED_GPS(1, :)); 

figure;
plot(a_NED_lpf(1, :)); hold on;
plot(a_NED_GPS(1, :)); 