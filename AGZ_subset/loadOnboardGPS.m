%% Import data from text file.
% ./AGZ/Log Files/OnbordGPS.csv
% This file contains the GPS data from the MAV onboard GPS receiver:
% 1: Timestemp, 2: imgid (id of the MAV image), 3: lat (Latitude in 1E7 degrees), 
% 4: lon (Longitude in 1E7 degrees), 5: alt (Altitude in 1E3 meters (millimeters) above MSL), 
% 6: s_variance_m_s (speed accuracy estimate m/s ), 7: c_variance_rad (course accuracy estimate rad), 
% 8: fix_type (0-1: no fix, 2: 2D fix, 3: 3D fix), 
% 9: eph_m (GPS HDOP horizontal dilution of position in m ), 
% 10: epv_m (GPS VDOP horizontal dilution of position in m ), 11: vel_n_m_s (GPS ground speed in m/s ), 
% 12: vel_e_m_s (GPS ground speed in m/s), 13: vel_d_m_s (GPS ground speed in m/s ), 
% 14: num_sat (Number of satellites visible.)
% more details: https://home.hibu.no/AtekStudenter1212/doxygen/bb_handler/structvehicle__gps__position__s.html

%% Initialize variables.
filename = 'Log Files/OnboardGPS.csv';
delimiter = ',';
startRow = 2;

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

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

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;