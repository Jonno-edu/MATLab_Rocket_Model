% Output setup
csv_filename = 'steve_sensor_data.csv';
mat_filename = 'data.mat';

% Relative path from simulation_6dof/scripts/utils to MATLAB_Toolbox/STEVE_Estimation_v2
output_folder = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'MATLAB_Toolbox', 'STEVE_Estimation_v2');

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

csv_fullpath = fullfile(output_folder, csv_filename);
mat_fullpath = fullfile(output_folder, mat_filename);

% Access logsout
logsout = simOut.logsout;

% Reference data
ref_Ve = logsout.getElement('ref_Ve');
ref_Ve_data = squeeze(ref_Ve.Values.Data);
ref_Ve_North = squeeze(ref_Ve_data(:,1));
ref_Ve_East = squeeze(ref_Ve_data(:,2));
ref_Ve_Down = squeeze(ref_Ve_data(:,3));

ref_Xe = logsout.getElement('ref_Xe');
ref_Xe_data = squeeze(ref_Xe.Values.Data);
ref_Xe_North = squeeze(ref_Xe_data(:,1));
ref_Xe_East = squeeze(ref_Xe_data(:,2));
ref_Xe_Down = squeeze(ref_Xe_data(:,3));

ref_q = squeeze(logsout.getElement('q_nb').Values.Data)';
ref_qw = squeeze(ref_q(:,1));
ref_qx = squeeze(ref_q(:,2));
ref_qy = squeeze(ref_q(:,3));
ref_qz = squeeze(ref_q(:,4));

% Sensor data
gps_Xe = logsout.getElement('gps_Xe');
gps_Xe_data = squeeze(gps_Xe.Values.Data)';
gps_Xe_North = squeeze(gps_Xe_data(:,1));
gps_Xe_East = squeeze(gps_Xe_data(:,2));
gps_Xe_Down = squeeze(gps_Xe_data(:,3));

gps_Ve = logsout.getElement('gps_Ve');
gps_Ve_data = squeeze(gps_Ve.Values.Data)';
gps_Ve_North = squeeze(gps_Ve_data(:,1));
gps_Ve_East = squeeze(gps_Ve_data(:,2));
gps_Ve_Down = squeeze(gps_Ve_data(:,3));

accelerometer = logsout.getElement('accelerometer');
accel_data = squeeze(accelerometer.Values.Data)';
accel_x = squeeze(accel_data(:,1));
accel_y = squeeze(accel_data(:,2));
accel_z = squeeze(accel_data(:,3));

gyro = logsout.getElement('gyro');
gyro_data = squeeze(gyro.Values.Data)';
gyro_x = squeeze(gyro_data(:,1));
gyro_y = squeeze(gyro_data(:,2));
gyro_z = squeeze(gyro_data(:,3));

mag = logsout.getElement('mag');
mag_data = squeeze(mag.Values.Data)';
mag_x = squeeze(mag_data(:,1));
mag_y = squeeze(mag_data(:,2));
mag_z = squeeze(mag_data(:,3));

% Time in milliseconds
time_ms = ref_Xe.Values.Time * 1e3;

% Create table
steve_sensor_data = table(time_ms, ...
    ref_Xe_North, ref_Xe_East, ref_Xe_Down, ...
    ref_Ve_North, ref_Ve_East, ref_Ve_Down, ...
    ref_qw, ref_qx, ref_qy, ref_qz, ...
    gps_Xe_North, gps_Xe_East, gps_Xe_Down, ...
    gps_Ve_North, gps_Ve_East, gps_Ve_Down, ...
    accel_x, accel_y, accel_z, ...
    gyro_x, gyro_y, gyro_z, ...
    mag_x, mag_y, mag_z, ...
    'VariableNames', {'time_ms', ...
                      'ref_Xe_North', 'ref_Xe_East', 'ref_Xe_Down', ...
                      'ref_Ve_North', 'ref_Ve_East', 'ref_Ve_Down', ...
                      'ref_qw', 'ref_qx', 'ref_qy', 'ref_qz', ...
                      'gps_Xe_North', 'gps_Xe_East', 'gps_Xe_Down', ...
                      'gps_Ve_North', 'gps_Ve_East', 'gps_Ve_Down', ...
                      'accel_x', 'accel_y', 'accel_z', ...
                      'gyro_x', 'gyro_y', 'gyro_z', ...
                      'mag_x', 'mag_y', 'mag_z'});

% Write CSV
writetable(steve_sensor_data, csv_fullpath);
fprintf('CSV written to %s\n', csv_fullpath);

% Save MAT file with table
save(mat_fullpath, 'steve_sensor_data');
fprintf('MAT file written to %s\n', mat_fullpath);
