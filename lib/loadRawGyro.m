%% Initialize variables.
filename = 'Log Files/RawGyro.csv';
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
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

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
timestamp = (dataArray{:, 1} - dataArray{1, 1}(1, 1)) ./ 1000000.0;
x = dataArray{:, 3};
y = dataArray{:, 4};
z = dataArray{:, 5};
temperature = dataArray{:, 6};
range_rad_s = dataArray{:, 7};
scaling = dataArray{:, 8};
x_raw = dataArray{:, 9};
y_raw = dataArray{:, 10};
z_raw = dataArray{:, 11};
temperature_raw = dataArray{:, 12};

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;