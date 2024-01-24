function drone = plotCam(drone, cam_num)
    %View angle range 
    width_ang = drone.camera(cam_num).fov(1)/2;
    length_ang = drone.camera(cam_num).fov(2)/2;

    %Update locations
    proj_x = [drone.X(3)*tan(-width_ang + drone.X(4)) + drone.X(1), drone.X(3)*tan(width_ang + drone.X(4)) + drone.X(1)];
    proj_y = [drone.X(3)*tan(-length_ang + drone.X(5)) + drone.X(2), drone.X(3)*tan(length_ang + drone.X(5)) + drone.X(2)];
    drone.camera(cam_num).proj_x = proj_x;
    drone.camera(cam_num).proj_y = proj_y;

    
    %plot3([proj_x(1),drone.X(1),proj_x(1),proj_x(end),drone.X(1),proj_x(end)], [proj_y(1), drone.X(2), proj_y(end),proj_y(end), drone.X(2), proj_y(1)], [0,drone.X(3),0,0,drone.X(3),0], 'k--');
    %plot3([proj_x(1), proj_x(1), proj_x(end), proj_x(end), proj_x(1)], [proj_y(1), proj_y(end), proj_y(end), proj_y(1), proj_y(1)], [0,0,0,0,0], 'k--');
end