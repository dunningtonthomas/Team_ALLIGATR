%% Component Builder
% Use this script to take in an excel sheet, exports an quantity index, and
% outputs all necessary fields of drone
% Author: Carson Kohlbrenner

% Housekeeping
clc; clear; close all;

% Load in the data from google sheets
data = readtable("Data/Component List - Raw Component List.csv");

% Data organizing
config_num = 2;
config(1).item_index_list = [1,1,5,6,6,7,7,16,17,17,17,17,18,18,18,18,19,20,21,22,23,24,25,26];
config(1).desc = "Drone using electronics shop cube, px4, telem radio";

electronics_shop_index_list = [28, 27];
config(2).item_index_list = [1,1,5,6,6,7,7,16,17,17,17,17,18,18,18,18,19,20,21,22,23,24,25,26, electronics_shop_index_list];
config(2).desc = "Full drone with no electronics shop parts";

%Analyze
config(config_num).cost = 0;
for i = 1:length(config(config_num).item_index_list)

        config(config_num).item_list(i).data = data(config(config_num).item_index_list(i), :);
        config(config_num).item_list(i).name = data{config(config_num).item_index_list(i), 1};
        config(config_num).item_list(i).cost = data{config(config_num).item_index_list(i), 6};

    %Cost
    config(config_num).cost = config(config_num).cost + config(config_num).item_list(i).cost;

    %Power

    %Weight
    
end