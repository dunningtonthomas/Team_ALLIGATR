% Uses the camera localization model to try to find the RGV
%Inputs:
%   -drone: drone object with its state at a given time
%   -Xf: [xf, yf] foot matrix in pixels of camera

function X = locate(drone, Xf)
    E = (2*rand([3,1])-1).*[drone.gps_e, drone.ang_e, drone.ang_e];
end