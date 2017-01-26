%% Import data from text file.
% Script for importing data from the following text file:
%
%    C:\Users\DRL\Documents\jugglerExperiment\Experiments\Mirror_Law\Juggler-Experiment_2017-01-22_13-55-25.pts
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2017/01/22 15:14:52

%% Initialize variables.
filename = 'Juggler-Experiment_2017-01-22_13-55-25.pts';
delimiter = '\t';
startRow = 2;

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
Frame = dataArray{:, 1};
x = dataArray{:, 2};
z = dataArray{:, 3};
xd = dataArray{:, 4};
zd = dataArray{:, 5};
psi = dataArray{:, 6};

frameRate = 120;
t = 0:length(Frame)-1;
t = t*1/frameRate;


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;