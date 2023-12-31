%% Clean Up
close all; clear; clc;

%% Localization Analysis
% Number of discrete positions for the UAS
numPos = 100; 
t = linspace(0, 100, numPos)';   % Parameterized variable
r = 10;                         % Radius in ft
height = 30;

% Define flight path of the UAS
zPos = height .* ones(numPos,1);    % Constant height
xPos = r * cos(2*pi* t / (100)) + 75;    % Orbital flight path
yPos = r * sin(2*pi* t / (100)) + 75;
UASPos = [xPos, yPos, zPos];

% Define arbitrary RGV position, stationary
zPosRGV = zeros(numPos, 1);  % On the ground
xPosRGV = 75 * ones(numPos, 1);
yPosRGV = 75 * ones(numPos, 1);
rgvPos = [xPosRGV, yPosRGV, zPosRGV];

% At each position along the flight path, calculate the range and pointing
% angle
range = sqrt((UASPos(:,1) - rgvPos(:,1)).^2 + (UASPos(:,2) - rgvPos(:,2)).^2 + (UASPos(:,3) - rgvPos(:,3)).^2);
angle = acosd(height ./ range);

% Add noise to the measurements 
meas = [range, angle];
noise = [1, 1.5];
noisyMeas = addNoise(meas, noise);

% Use pointing angle to get the estimation on the ground of the location of
% the RGV




%% Plotting
figure();
plot3(UASPos(:,1), UASPos(:,2), UASPos(:,3), 'linewidth', 2);
hold on;
grid on;
plot3(rgvPos(1), rgvPos(2), 0, '.', 'MarkerSize', 20)

%2D plot of estimation
figure();
plot(rgvPos(1), rgvPos(2), '.', 'MarkerSize', 20);
grid on;
hold on;




%% Functions
%This function adds noise to the inputted measurements
function out = addNoise(meas, noise)
    % meas is a nx2 matrix with the first column corresponding to the first
    % measurement and the seconds column is the second
    % noise is 2x1, noise(1) is the largest noise value for meas1 and
    % noise(2) is for meas2
    
    out = zeros(length(meas(:,1)), 2);
    for i = 1:length(meas(:,1))
        % Calculate pseudorandom noise
        noise1 = (rand() * 2 - 1) * noise(1);
        noise2 = (rand() * 2 - 1) * noise(2);

        % Calculate the new measurements
        out(i,1) = meas(i,1) + noise1;
        out(i,2) = meas(i,2) + noise2;
    end
end




