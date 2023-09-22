%% Component Builder
% Use this script to take in an excel sheet, exports an quantity index, and
% outputs all necessary fields of drone
% Author: Carson Kohlbrenner

% Housekeeping
clc; clear; close all;

% Load in the data from google sheets
data = readcell("Data/Subteams Component List - Hardware.csv");

% Data organizing

