%% Clean Up
close all; clear; clc;

%% Initialization
%Sensor Characteristics
range = 30;     %ft, height of the drone
FOV = 45;      %deg

%Simulation time
tStep = 0.1;    %s

%Define the ground side length
bound = 150;

%Define the radius of the field of view
radius = range*tand(FOV);

%Define the position of the drone
center = [75; 75];

%Initial FOV area
angles = linspace(0, 2*pi, 100);
FOVx = radius * cos(angles) + center(1);
FOVy = radius * sin(angles) + center(2);

%Initial RGV Position
RGVPos1 = [bound*rand(1); bound*rand(1)]; %Random initial position
RGVPos2 = [bound*rand(1); bound*rand(1)]; %Random initial position

%Drone positions defined by search algorithm
sizePos = bound / 2 / tStep;
pos1 = [linspace(0, 25, 100); linspace(0, 25, 100)];
pos2 = [linspace(25,bound-10, 400); 25.*ones(1,400)];
pos3 = [(bound-10).*ones(1,100); linspace(25,50,100)];
pos4 = [linspace(bound-10, 25, 400); 50.*ones(1,400)];
pos5 = [25.*ones(1,100); linspace(50,75,100)];
pos6 = [linspace(25, bound-10, 400); 75.*ones(1,400)];
pos7 = [(bound-10).*ones(1,100); linspace(75,100,100)];
pos8 = [linspace(bound-10, 25, 400); 100.*ones(1,400)];
pos9 = [25.*ones(1,100); linspace(100,125,100)];
pos10 = [linspace(25, bound-10, 400); 125.*ones(1,400)];
pos11 = [(bound-10).*ones(1,100); linspace(125,150,100)];


dronePositions1 = [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10, pos11];
dronePositions2 = flip(dronePositions1,2);
dronePositions = [dronePositions1, dronePositions2];
[~,searchLength] = size(dronePositions);

%% Simulation
%Initial Drone position
figure();
fovPlot = plot(FOVx, FOVy, 'linewidth', 2);
hold on;
grid on;
RGVPlot1 = plot(RGVPos1(1), RGVPos1(2), '.', 'color', 'g');
RGVPlot2 = plot(RGVPos2(1), RGVPos2(2), '.', 'color', 'g');
ylim([-25 bound + 25]);
xlim([-25 bound + 25]);

%Draw the boundary
boundary = [0, bound, bound, 0, 0; 0, 0, bound, bound, 0];
plot(boundary(1,:), boundary(2,:), 'linewidth', 1, 'color', 'k', 'LineStyle','--');

%Simulation Loop
detect = false;
iteration = 1;
while(true)

    %Calculate a new Drone position
    newDrone = dronePositions(:, iteration);

    %Determine new FOV
    FOVx = radius * cos(angles) + newDrone(1);
    FOVy = radius * sin(angles) + newDrone(2);

    %Calculate difference between drone position and RGV position
    diff1 = norm(RGVPos1 - newDrone);
    diff2 = norm(RGVPos2 - newDrone);

    %Check if it is within the FOV
    if(diff1 <= radius)
        colorRGV1 = 'g';
    else
        colorRGV1 = 'r';
    end

    if(diff2 <= radius)
        colorRGV2 = 'g';
    else
        colorRGV2 = 'r';
    end


    %Plot the new positions
    delete(fovPlot);
    delete(RGVPlot1);
    delete(RGVPlot2);
    %fovPlot = plot(FOVx, FOVy, 'linewidth', 2, 'color', 'b');
    fovPlot = fill(FOVx, FOVy, 'b', 'FaceAlpha', 0.2, 'EdgeColor','none');
    RGVPlot1 = plot(RGVPos1(1), RGVPos1(2), '.', 'color', colorRGV1, 'MarkerSize', 20);
    RGVPlot2 = plot(RGVPos2(1), RGVPos2(2), '.', 'color', colorRGV2, 'MarkerSize', 20);

    %Animate
    drawnow;

    %Update the iteration of the search algorithm
    iteration = iteration + 1;
    if(iteration > searchLength)
        iteration = 1; %Restart the algorithm
    end

    %Sleep
    %pause(tStep/50);
end









