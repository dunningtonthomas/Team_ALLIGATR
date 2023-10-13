function [path] = makePath(xBoundary, yBoundary, width, speed)
% This function takes in the x and y boundary and creates a grid search
% path, returning an array. 
% Width is the width of the FOV circle
% Speed is the ft/s rate of drone, want to take 1 step every second
    
    path = [];

    % Get limits
    xMin = min(xBoundary);
    xMax = max(xBoundary);
    xDist = xMax - xMin;
    yMin = min(yBoundary);
    yMax = max(yBoundary);
    yDist = yMax - yMin;

    % Decide the number of passes in y direction
    % numX = ceil((xMax - xMin)/width);
    numY = ceil(yDist/width);
    
    % will make grid path so that loops through every x for numY passes,
    % seperated width apart
    yVec = linspace(yMin,yMax,ceil(yDist/width)+1);
    for i = 1:length(yVec)
        xVec = linspace(xMin,xMax,ceil(xDist/speed)+1); % ceil limits to under the desired speed
        yCurr = yVec(i);
        if mod(i,2) == 0
            % need to reverse the direction of x
            matCurr = [flip(xVec); ones(size(xVec))*yCurr];
        else
            matCurr = [xVec; ones(size(xVec))*yCurr];
        end
        path = [path, matCurr];
    end
end

