function drone = drone_params()
% Mission parameters
    drone.flight_bounds = [30, 60]; %ft

% Drone State
    drone.x_0 = 0;
    drone.y_0 = 0;
    drone.z_0 = 30; %ft

    drone.roll_0 = 0;
    drone.pitch_0 = pi/16;
    drone.yaw_0 = 0;

    drone.u_0 = 0;
    drone.v_0 = 0;
    drone.w_0 = 0;

    drone.p_0 = 0;
    drone.q_0 = 0;
    drone.r_0 = 0;

    drone.X = [drone.x_0; drone.y_0; drone.z_0; drone.pitch_0; drone.yaw_0; drone.roll_0; ...
               drone.u_0; drone.v_0; drone.w_0; drone.p_0; drone.q_0; drone.r_0];



%%%%%%%%%%% Camera Entry 1 %%%%%%%%%%%%%%%
    drone.camera(1).name = "Camera Module 3 Wide";

    % Camera FOV [horz_fov, vert_fov, total_fov]
    drone.camera(1).fov = [102, 67, 120];

    % Camera resolutions for video [res_x, res_y, p]
    drone.camera(1).vid_res = [2304, 1296, 56; ...
                    2304, 1296, 30; ...
                    1536, 864, 120];

    % Camera resolutions for still shots [res_x, res_y]
    drone.camera(1).still_res = [4608, 2592];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end