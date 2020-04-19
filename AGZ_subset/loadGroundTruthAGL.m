%% Import data from text file.
% ./AGZ/Log Files/GroundTruthAGL.csv
% 1: imgid (id of the MAV image), 2: x_gt (ground truth camera position x), 
% 3: y_gt (ground truth camera position y), 4: z_gt (ground truth camera position z), 
% 5: omega_gt (degrees, ground truth camera orientation yaw), 
% 6: phi_gt (degrees, ground truth camera orientation pith), 
% 7: kappa_gt (degrees, ground truth camera orientation roll), 
% 8: x_gps (GPS camera position x), 9: y_gps (GPS camera position y), 10: z_gps (GPS camera position z)
% All values regarding positions are in the WGS 84 / UTM zone 32N coordinate system use plotPath.m to visualize the data in matlab

%% Initialize variables.
filename = 'Log Files/GroundTruthAGL.csv';
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
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

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
imgid = dataArray{:, 1};
x_gt = dataArray{:, 2};
y_gt = dataArray{:, 3};
z_gt = dataArray{:, 4};
omega_gt = dataArray{:, 5};
phi_gt = dataArray{:, 6};
kappa_gt = dataArray{:, 7};
x_gps = dataArray{:, 8};
y_gps = dataArray{:, 9};
z_gps = dataArray{:, 10};

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;