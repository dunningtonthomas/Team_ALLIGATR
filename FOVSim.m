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
RGVPos = [75; 75];

%Drone positions defined by search algorithm
sizePos = bound / 2 / tStep;
dronePositions = [linspace(0, 150, sizePos); linspace(0, 150, sizePos)]; %This will come from search algorithm

%Get RGV positions for the length of the simulation
RGVPositions = [linspace(75, 120, sizePos/2), linspace(120, 75, sizePos/2); linspace(75, 115, sizePos/2), linspace(115, 150, sizePos/2)];

%% Simulation
%Initial Drone position
figure();
fovPlot = plot(FOVx, FOVy, 'linewidth', 2);
hold on;
grid on;
RGVPlot = plot(RGVPos(1), RGVPos(2), '.', 'color', 'g');
ylim([0 bound]);
xlim([0 bound]);

%Simulation Loop
detect = false;
iteration = 1;
while(true)
    %Calculate a RGV_Pos
    RGVPos = RGVPositions(:, iteration);

    %Calculate a new Drone position
    newDrone = dronePositions(:, iteration);

    %Determine new FOV
    FOVx = radius * cos(angles) + newDrone(1);
    FOVy = radius * sin(angles) + newDrone(2);

    %Calculate difference between drone position and RGV position
    diff = norm(RGVPos - newDrone);

    %Check if it is within the FOV
    if(diff <= radius)
        detect = true; %In frame
        colorRGV = 'g';
    else
        detect = false; %Out of frame
        colorRGV = 'r';
    end


    %Plot the new positions
    delete(fovPlot);
    delete(RGVPlot);
    fovPlot = plot(FOVx, FOVy, 'linewidth', 2, 'color', 'b');
    RGVPlot = plot(RGVPos(1), RGVPos(2), '.', 'color', colorRGV, 'MarkerSize', 20);

    %Animate
    drawnow;

    %Update the iteration of the search algorithm
    iteration = iteration + 1;
    if(iteration > sizePos)
        iteration = 1; %Restart the algorithm
    end

    %Sleep
    pause(tStep/50);
end









