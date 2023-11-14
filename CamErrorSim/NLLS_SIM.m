clc; close all; clear;

%This script will compare the errors for different video resolutions and
%focal lengths
drone = drone_params();

%Define the equations of motion for the RGV
rgv = rgv_params();

%Select Camera to analyze in this sim
cam_num = 1;

% Simulation parameters
fps = 24;
sim_duration = 6; %s
steps = fps*sim_duration;

% RGV controllers
dt = 1 / fps;

RGVforwardEOM = @(X, dt) [X(1); X(2) + rgv.max_speed*dt; X(3)];

RGVstillEOM = @(X, dt) [X(1); X(2); X(3)];

%Drone Controllers
droneFollowEOM = @(rgv, drone) [rgv.X(1); rgv.X(2); rgv.X(3) + drone.X(3); drone.X(4); drone.X(5); drone.X(6); ...
                                drone.X(7); drone.X(8); drone.X(9); drone.X(10); drone.X(11); drone.X(12)];
droneHoverEOM = @(rgv, drone) [drone.X(1); drone.X(2); drone.X(3); drone.X(4); drone.X(5); drone.X(6); ...
                                drone.X(7); drone.X(8); drone.X(9); drone.X(10); drone.X(11); drone.X(12)];
radius = 20;
droneCircleEOM = @(drone, t) [radius*sin(t); radius*cos(t); drone.X(3); drone.X(4); drone.X(5); drone.X(6); ...
                                drone.X(7); drone.X(8); drone.X(9); drone.X(10); drone.X(11); drone.X(12)];

%Measurement errors
noise_range = 6; %ft

% Simulate
figure();
rgvPath = zeros(3,steps);
rgvMeasurments = zeros(3,steps);
dronePath = zeros(12,steps);

for i = 1:steps
    t = i/fps;
    %Equations of motion iterated
    rgv.X = RGVstillEOM(rgv.X, dt);
    %drone.X = droneFollowEOM(rgv, drone);
    drone.X = droneCircleEOM(drone,t);
    rgvPath(:,i) = rgv.X;
    dronePath(:,i) = drone.X;

    %Simulate noise in a measurement
    r = (2*noise_range.*rand(2,1)) - noise_range;
    rgvMeasurments(:,i) = rgv.X + [r;0];

    %Plot for each step
    plot3(rgvPath(1,1:i), rgvPath(2,1:i), rgvPath(3,1:i), 'LineWidth', 3);
    hold on;
    axis equal;
    grid on;
    plot3(dronePath(1,1:i), dronePath(2,1:i), dronePath(3,1:i), 'LineWidth', 3);
    scatter3(rgvMeasurments(1,1:i), rgvMeasurments(2,1:i), rgvMeasurments(3,1:i), 10, 'filled');
    drone = plotCam(drone, cam_num);
    hold off;

    drawnow;
%     pause(0.2);
end

figure();
hold on;
axis equal;
grid on;
scatter(rgvMeasurments(1,:) - dronePath(1,:), rgvMeasurments(2,:) - dronePath(2,:), 'filled');
plot(rgvPath(1,:) - dronePath(1,:), rgvPath(2,:) - dronePath(2,:));
legend(["Measured Pos", "True Path"]);
xlabel("X (ft)");
ylabel("Y (ft)");
title("Relative Measurements");
hold off;

figure();
hold on;
axis equal;
grid on;
scatter(rgvMeasurments(1,:), rgvMeasurments(2,:), 'filled');
plot(rgvPath(1,:), rgvPath(2,:));
legend(["Measured Pos", "True Path"]);
xlabel("X (ft)");
ylabel("Y (ft)");
title("Inertial Measurements");
hold off;




