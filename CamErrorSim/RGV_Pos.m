function [newPos] = RGV_Pos(currPos, bound, t)
%RGV_POS This function takes in the current position of the RGV and and
%randomizes its next position and remains within the 150 by 150 ft area
%t is the current time step

    temp = true;
    while(temp)
        %Velocity
        vel = 2; %Assume maximum velocity
    
        %Propagate random location, generate rand numbers between -0.5 and 0.5
        randX = rand(1) - 0.5;
        randY = rand(1) - 0.5;
    
        %Calculate new position
        newPos = currPos + vel*t*[randX; randY];
    
        %Check if the new position is outside the bounds
        if(~(sum(newPos <= 0) || sum(newPos >= bound)))
            break; %Exit loop 
        end
    end

end

