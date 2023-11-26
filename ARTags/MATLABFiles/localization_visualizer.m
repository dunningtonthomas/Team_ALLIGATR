%% Clean up
close all; clear; clc;

%% Import Data
path = "../OutputFiles/";
data = readmatrix(path + "localization_1.csv");

%% Analysis
% Drone Position
drone_E = data(:,2);
drone_N = data(:,3);

% RGV Position
rgv_E = data(:,4);
rgv_N = data(:,5);

% IDs
ids = data(:,6);

% Number of frames
frames = length(data(:,1));

% Calculate the estimated RGV position
rgvA_pos = zeros(1,2);
rgvB_pos = zeros(1,2);
A = 1;
B = 1;
for i = 1:frames
    if ids(i) ~= 0          % Detected RGV
        if ids(i) == 1  
            rgvA_pos(A, :) = [rgv_E(i), rgv_N(i)]; % RGV A
            A = A + 1; % Update iterator
        else            
            rgvB_pos(B, :) = [rgv_E(i), rgv_N(i)]; % RGV B
            B = B + 1; % Update iterator
        end
    end
end

% Use the averages for the estimated positions
rgvA_Final = mean(rgvA_pos, 1);
rgvB_Final = mean(rgvB_pos, 1);



%% Plotting
fig1 = figure();
% Animated line
droneLine = animatedline(drone_E(1), drone_N(1), 'Color', 'k', 'LineWidth', 1.5);
hold on;
grid on;
xlim([-21 1])
ylim([-12 6])
xlabel('East (m)');
ylabel('North (m)');
title('Localization Test')
droneFrames(1) = getframe(fig1);

% Loop through each frame
for i = 2:frames

    % Plot the current drone position
    addpoints(droneLine,drone_E(i),drone_N(i));
    drawnow

    % Plot rgv if visible
    if rgv_E(i) == 0 && rgv_N(i) == 0    % RGV not visible
        % Save frame
        droneFrames(i) = getframe(fig1);
        continue
    elseif ids(i) == 1 
        plot(rgv_E(i), rgv_N(i), '.', 'color','r') % RGV A
    else
        plot(rgv_E(i), rgv_N(i), '.', 'color','b') % RGV B
    end

    % Save frame
    droneFrames(i) = getframe(fig1);

end


%% RGV A
fig2 = figure();
plot(0,0);
hold on
grid on
xlim([-13 -12])
ylim([-10 -9.5])
xlabel('East (m)');
ylabel('North (m)');
title('RGV A')

for i = 1:frames
    % Plot the estimate
    if i == frames
        plot(rgvA_Final(1), rgvA_Final(2), '.', 'color','k', 'MarkerSize', 25)
    end

    % Plot rgv if visible
    if rgv_E(i) == 0 && rgv_N(i) == 0    % RGV not visible
        % Save the frame
        rgvAFrames(i) = getframe(fig2);
        continue
    end

    % Plot position
    plot(rgv_E(i), rgv_N(i), '.', 'color','r', 'MarkerSize', 20)

    % Animate
    drawnow

    % Save the frame
    rgvAFrames(i) = getframe(fig2);
end



%% RGV B
fig3 = figure();
plot(0, 0)
hold on
grid on
xlim([-17 -16.5])
ylim([-1.5 -1])
xlabel('East (m)');
ylabel('North (m)');
title('RGV B')

for i = 1:frames

    % Plot the estimate
    if i == frames
        plot(rgvB_Final(1), rgvB_Final(2), '.', 'color','k', 'MarkerSize', 25)
    end

    % Plot rgv if visible
    if rgv_E(i) == 0 && rgv_N(i) == 0    % RGV not visible
        % Save the frame
        rgvBFrames(i) = getframe(fig3);
        continue
    end

    % Plot position
    plot(rgv_E(i), rgv_N(i), '.', 'color','b', 'MarkerSize', 20)
    
    % Animate
    drawnow

    % Save the frame
    rgvBFrames(i) = getframe(fig3);
end


%% Save videos to files
% %Save animation to a file
% droneWrite = VideoWriter('dronePos');
% droneWrite.FrameRate = 30;
% open(droneWrite);
% writeVideo(droneWrite, droneFrames);
% close(droneWrite);

% RGV A
rgvAWrite = VideoWriter('rgvAPos');
rgvAWrite.FrameRate = 30;
open(rgvAWrite);
writeVideo(rgvAWrite, rgvAFrames);
close(rgvAWrite);

% RGV B
rgvBWrite = VideoWriter('rgvBPos');
rgvBWrite.FrameRate = 30;
open(rgvBWrite);
writeVideo(rgvBWrite, rgvBFrames);
close(rgvBWrite);





