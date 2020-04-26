%% 
% start from position (0, 0, 0)

%% load raw gps lat, lon, alt into cartesian x,y,z 
% ./AGZ/Log Files/OnbordGPS.csv

%% Initialize variables.
filename = './AGZ_subset/Log Files/OnboardGPS.csv';
delimiter = ',';
startRow = 2;

% add lib path
addpath([cd, filesep, 'lib']);

%% load raw data
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

%% Allocate imported array to column variable names
timestamp = dataArray{:, 1};
lat = dataArray{:, 3};
lon = dataArray{:, 4};
alt = dataArray{:, 5};
% temperature = dataArray{:, 6};
% range_rad_s = dataArray{:, 7};
% scaling = dataArray{:, 8};
% x_raw = dataArray{:, 9};
% y_raw = dataArray{:, 10};
% z_raw = dataArray{:, 11};
% temperature_raw = dataArray{:, 12};

len = size(lat);
GPS = zeros(len(1), 4);
[x, y, z] = gps2cart(lat(1), lon(1), alt(1));
initial = [timestamp(1) / 1000000, x, y, z];
for i = 1:len(1)
    ti = timestamp(i) / 1000000;
    la = lat(i);
    lo = lon(i);
    al = alt(i);
    [x, y, z] = gps2cart(la, lo, al);
    GPS(i,:) = [ti, x, y, z] - initial;
end

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;