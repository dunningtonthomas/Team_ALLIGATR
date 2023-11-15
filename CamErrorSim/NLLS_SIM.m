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
sim_duration = 10; %s
steps = fps*sim_duration;

% RGV controllers
dt = 1 / fps;
t = linspace(0,sim_duration, steps);

% Equations of motion (Ambiguous to the vehicle)
forwardEOM = @(obj, X, t) [X(1) + obj.max_speed*t; X(2)*ones(size(t)); X(3)*ones(size(t))];
stillEOM = @(obj, X, t) [X(1)*ones(size(t)); X(2)*ones(size(t)); X(3)*ones(size(t))];
hoverEOM = @(obj, X, t) [X(1)*ones(size(t)); X(2)*ones(size(t)); X(3)*ones(size(t))];
circleEOM = @(obj, X, t) [X(1) + obj.radius*sin(t); X(2) + obj.radius*cos(t); X(3)*ones(size(t))];
sinEOM = @(obj, X, t) [X(1) + obj.radius*sin(t)+ obj.max_speed*t; X(2) + obj.max_speed*t; X(3)*ones(size(t))];
cosEOM = @(obj, X, t) [X(1) + obj.max_speed*t + obj.radius*sin(t); X(2) + obj.radius*cos(t).^3; X(3)*ones(size(t))];

%Measurement errors
noise_range = 12; %ft

% Simulate
f1 = figure();
rgvMeasurements = zeros(3,steps);
y_k = [];

%Calculate path and set initial conditions
rgvEOM = sinEOM;
droneEOM = stillEOM;

rgvPath = rgvEOM(rgv, rgv.X, t);
dronePath = droneEOM(drone, drone.X, t);

for i = 1:steps
    %Save the state for each timestep
    drone.X(1:3) = dronePath(:,i);
    rgv.X = rgvPath(:,i);

    %Simulate noise in a measurement
    r = (2*noise_range.*rand(2,1)) - noise_range;
    rgvMeasurements(:,i) = rgv.X + [r;0];
    y_k = [y_k;rgvMeasurements(:,i)];

    %Plot for each step
    plot3(rgvPath(1,1:i), rgvPath(2,1:i), rgvPath(3,1:i), 'LineWidth', 3);
    hold on;
    axis equal;
    grid on;
    plot3(dronePath(1,1:i), dronePath(2,1:i), dronePath(3,1:i), 'LineWidth', 3);
    scatter3(rgvMeasurements(1,1:i), rgvMeasurements(2,1:i), rgvMeasurements(3,1:i), 10, 'filled');
    drone = plotCam(drone, cam_num);
    hold off;

    drawnow;
%     pause(0.2);
end

%Calculate the non-linear least squares for cureve fitting
[R_pred, path] = NLS(y_k, rgvEOM, droneEOM, rgvMeasurements(:,1), drone.X0, t);
rgvPredPath = rgvEOM(rgv, R_pred, t) - droneEOM(drone, drone.X0, t);
rgvPredInerPath = rgvEOM(rgv, R_pred, t);

%Display the error in ft
fprintf("X error: %0.2f ft\n", abs(R_pred(1)-rgv.X0(1)));
fprintf("Y error: %0.2f ft\n", abs(R_pred(2)-rgv.X0(2)));
fprintf("Z error: %0.2f ft\n", abs(R_pred(3)-rgv.X0(3)));
fprintf("Total Error: %0.2f ft\n", norm(R_pred-rgv.X0));

%PLace the predicted path on the final sim
figure(f1);
hold on;
plot3(rgvPredInerPath(1,:), rgvPredInerPath(2,:), rgvPredInerPath(3,:), 'k', 'LineWidth',4);
hold off;


%Plot the relative RGV positons relative to the drone
figure();
hold on;
axis equal;
grid on;
scatter(rgvMeasurements(1,:) - dronePath(1,:), rgvMeasurements(2,:) - dronePath(2,:), 'filled');
%scatter(path(1,:) - dronePath(1,1), path(2,:) - dronePath(2,1), 'kx');
plot(rgvPath(1,:) - dronePath(1,:), rgvPath(2,:) - dronePath(2,:));
scatter(rgvPredPath(1,:), rgvPredPath(2,:), 'kx');
legend(["Measured Pos", "True Path", "Predicted Path"]);
xlabel("X (ft)");
ylabel("Y (ft)");
title("Relative Measurements");
hold off;

%Plot the RGV positons in the inertial frame
figure();
hold on;
axis equal;
grid on;
scatter(rgvMeasurements(1,:), rgvMeasurements(2,:), 'filled');
%scatter(path(1,:), path(2,:), 'kx');
plot(rgvPath(1,:), rgvPath(2,:));
scatter(rgvPredInerPath(1,:), rgvPredInerPath(2,:), 'kx');
legend(["Measured Pos", "True Path", "Predicted Path"]);
xlabel("X (ft)");
ylabel("Y (ft)");
title("Inertial Measurements");
hold off;




