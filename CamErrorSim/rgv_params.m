function rgv = rgv_params()
    % Movement Parameters
    rgv.max_speed = 2*3.28084 * rand(); % ft/s
    rgv.radius = 10; %ft, turn radius

    % Intitial state
    rand_range = 10; %ft
    rgv.x_0 = ((rand()-0.5)*rand_range);
    rgv.y_0 = ((rand()-0.5)*rand_range);
    rgv.z_0 = 0;

    % Current State
    rgv.X = [rgv.x_0; rgv.y_0; rgv.z_0];
    rgv.X0 = rgv.X;
end