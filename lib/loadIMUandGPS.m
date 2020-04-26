%% README
% This script loads IMU data into 
% imu = [ts, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z],
% It loads GPS data into
% gps = [ts, gps_x, gps_y, gps_z]
% It loads Ground Truth Poses into
% gt = [ts, x_gt, y_gt, z_gt, omega_gt, phi_gt, kappa_gt];
% NOTE: timestamps of IMU and GPS are not matched
% After running this script, you can load IMU and GPS data directly from 
% 'IMU_GPS_GT_data.mat'

clear;
dir = '../AGZ_subset/';
%% load Accel.
filename = append(dir , 'Log Files/RawAccel.csv');
delimiter = ',';
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

fileID = fopen(filename,'r');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

fclose(fileID);

init_ts_accel = dataArray{1, 1}(1, 1);
timestamp = (dataArray{:, 1} - dataArray{1, 1}(1, 1)) ./ 1000000.0;
x = dataArray{:, 3};
y = dataArray{:, 4};
z = dataArray{:, 5};

accel = [timestamp, x, y, z];

clearvars filename delimiter startRow formatSpec fileID dataArray ans;



%% load Gyro.
filename = append(dir , 'Log Files/RawGyro.csv');
delimiter = ',';
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

fileID = fopen(filename,'r');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

fclose(fileID);

init_ts_gyro = dataArray{1, 1}(1, 1);
timestamp = (dataArray{:, 1} - dataArray{1, 1}(1, 1)) ./ 1000000.0;
x = dataArray{:, 3};
y = dataArray{:, 4};
z = dataArray{:, 5};

gyro = [timestamp, x, y, z];

imu = [accel, x, y, z];

clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%% load Cartesian GPS.
filename = append(dir , 'Log Files/OnboardGPS.csv');
delimiter = ',';
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

init_ts_gps = dataArray{1, 1}(1, 1);
timestamp = dataArray{:, 1};
lat = dataArray{:, 3};
lon = dataArray{:, 4};
alt = dataArray{:, 5};

len = size(lat);
gps = zeros(len(1), 4);
[x, y, z] = gps2cart(lat(1), lon(1), alt(1));
initial = [timestamp(1) / 1000000, x, y, z];
for i = 1:len(1)
    ti = timestamp(i) / 1000000;
    la = lat(i);
    lo = lon(i);
    al = alt(i);
    [x, y, z] = gps2cart(la, lo, al);
    gps(i,:) = [ti, x, y, z] - initial;
end

clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%% Process timestamps
init_ts = min([init_ts_accel, init_ts_gyro, init_ts_gps]);
accel(:, 1) = accel(:, 1) + (init_ts_accel - init_ts) / 1000000;
gyro(:, 1) = gyro(:, 1) + (init_ts_gyro - init_ts) / 1000000;
gps(:, 1) = gps(:, 1) + (init_ts_gps - init_ts) / 1000000;
imu(:, 1) = imu(:, 1) + (init_ts_accel - init_ts) / 1000000;




%% load Ground Truth.
filename = append(dir , 'Log Files/GroundTruthAGL.csv');
delimiter = ',';
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

fileID = fopen(filename,'r');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

fclose(fileID);

imgid = dataArray{:, 1};
x_gt = dataArray{:, 2};
y_gt = dataArray{:, 3};
z_gt = dataArray{:, 4};
omega_gt = dataArray{:, 5};
phi_gt = dataArray{:, 6};
kappa_gt = dataArray{:, 7};

ts = [];
for i = 1:length(imgid)
    ts = [ts; gps(imgid(i))];
end

gt = [ts, x_gt, y_gt, z_gt, omega_gt, phi_gt, kappa_gt];

clearvars filename delimiter startRow formatSpec fileID dataArray ans;


%% save data
Filename = sprintf('IMU_GPS_GT_data.mat');
save(Filename, 'gps', 'imu', 'gt');

