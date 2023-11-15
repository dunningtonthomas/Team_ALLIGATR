function rgv = rgv_params()
    % Movement Parameters
    rgv.max_speed = 2*3.28084; % ft/s
    rgv.radius = 10; %ft, turn radius

    % Intitial state
    rgv.x_0 = 0;
    rgv.y_0 = 0;
    rgv.z_0 = 0;

    % Current State
    rgv.X = [rgv.x_0; rgv.y_0; rgv.z_0];
    rgv.X0 = rgv.X;
end