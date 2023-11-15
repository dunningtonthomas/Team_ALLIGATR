clc; close all; clear;

%This script will compare the errors for different video resolutions and
%focal lengths

%%%%%%%%%%% Camera Entry 1 %%%%%%%%%%%%%%%
    camera(1).name = "Camera Module 3 Wide";

    % Camera FOV [horz_fov, vert_fov, total_fov]
    camera(1).fov = [102, 67, 120];

    % Camera resolutions for video [res_x, res_y, p]
    camera(1).vid_res = [2304, 1296, 56; ...
                    2304, 1296, 30; ...
                    1536, 864, 120];

    % Camera resolutions for still shots [res_x, res_y]
    camera(1).still_res = [4608, 2592];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% Camera Entry 2 %%%%%%%%%%%%%%%
    camera(2).name = "Camera Module 3 Narrow";

    % Camera FOV [horz_fov, vert_fov, total_fov]
    camera(2).fov = [66, 41, 75];

    % Camera resolutions for video [res_x, res_y, p]
    camera(2).vid_res = [2304, 1296, 56; ...
                    2304, 1296, 30; ...
                    1536, 864, 120];

    % Camera resolutions for still shots [res_x, res_y]
    camera(2).still_res = [4608, 2592];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Select Camera
%cam_num = 2;

%Jackal Specs
rgv_A_h = 1.67;
rgv_A_v = 1.41;
rgv_A = 1.67 * 1.41; %ft^2
min_pixels = 8*8; %pixels

% Flight Parameters
flight_bounds = [30, 60]; %ft elevation

%For 2D projects, quantify horizontal and vertical angle straight down from
%drone
camera_angle = [0,0]; %[roll_angle, pitch_angle, yaw_angle]

%Labels
plotFigs = false;
axes_labels = ["Horizontal", "Vertical"];
err_lim = [0,0.2]; %ft err
c = jet;

    %Check the video accuracy per pixel
for cam_num = 1:2
    for fb = flight_bounds
        fprintf("\n <strong> %s at %0.1f ft high: </strong>\n",camera(cam_num).name, fb);
        for setting = 1:size(camera(cam_num).vid_res, 1)
            fprintf(" - Using video resolution of %0.0ix%0.0i %0.0ip:\n", camera(cam_num).vid_res(setting, 1), camera(cam_num).vid_res(setting, 2), camera(cam_num).vid_res(setting, 3));
            
            clear pixel_x pixel_err;
            for axes = 1:2
                fov = camera(cam_num).fov(axes);
                res = camera(cam_num).vid_res(setting, axes);
                start_angle = -fov/2 + camera_angle(axes);
                end_angle = fov/2 + camera_angle(axes);
                d_angle = fov/res;
                chi = start_angle:d_angle:end_angle;
    
                %Calculate the resolution for each pixel
                for i = 2:length(chi)
                    %camera(cam_num).pixel_err(setting, axes, i) = abs(fb*tan(deg2rad(chi(i))) - fb*tan(deg2rad(chi(i-1))));
                    pixel_err(axes, i) = abs(fb*tan(deg2rad(chi(i))) - fb*tan(deg2rad(chi(i-1))));
                    
                    %camera(cam_num).pixel_x(setting, axes, i) = fb*tan(deg2rad(chi(i)));
                    pixel_x(axes, i) = fb*tan(deg2rad(chi(i)));
                end
    
                clear px_err;
                px_err = pixel_err(axes,2:end);
                px_err(px_err == 0) = [];
                min_err(axes) = min(px_err);
                max_err(axes) = max(px_err);
    
                %Output Results
                fprintf("        >> %s error range of %0.3f - %0.3f ft/pixel. Jackal %s in <strong> %0.0f - %0.0f </strong> pixels\n", axes_labels(axes), min(px_err), max(px_err), axes_labels(axes), sqrt(rgv_A)/min_err(axes), sqrt(rgv_A)/max_err(axes));
            end
    
            fprintf("        >> %0.1f ft^2 blob seen in <strong> %0.0f - %0.0f </strong> pixels\n\n", rgv_A, rgv_A / (min_err(1)*min_err(2)), rgv_A / (max_err(1)*max_err(2)) );
    
            if plotFigs
                figure();
        
                % Horizontal FOV plot
                subplot(2,1,1);
                hold on;
                axis equal;
                % Get the x points on the graph
                pe_x = pixel_err(1,:);
                px = pixel_x(1,:);
                px = px(pe_x>0);
                pe_x = pe_x(pe_x>0);
        
                plot([px(1), 0, px(end)], [0,fb,0], 'k--');
                scatter(0, fb, 50, 'k', 'filled');
                scatter(px, zeros(size(px)), 10, pe_x, 'filled');
                title(camera(cam_num).fov(1) + "° HFOV, " + camera(cam_num).vid_res(setting, 1) + " pixels");
                xlabel("ft");
                ylabel("ft");
                colorbar;
                clim(err_lim);
                colormap(c);
                hold off;
        
                % Vertical FOV plot
                subplot(2,1,2);
                hold on;
                axis equal;
                % Get the x points on the graph
                pe_y = pixel_err(2,:);
                py = pixel_x(2,:);
                py = py(pe_y>0);
                pe_y = pe_y(pe_y>0);
        
                plot([py(1), 0, py(end)], [0,fb,0], 'k--');
                scatter(0, fb, 50, 'k', 'filled');
                scatter(py, zeros(size(py)), 10, pe_y, 'filled')
                title(camera(cam_num).fov(2) + "° VFOV, " + camera(cam_num).vid_res(setting, 2) + " pixels");
                xlabel("ft");
                ylabel("ft");
                colorbar;
                clim(err_lim);
                colormap(c);
                hold off;
                sgtitle(camera(cam_num).name + " at " + fb + "ft - " + camera(cam_num).vid_res(setting, 1) + "x" + camera(cam_num).vid_res(setting, 2) + " " + camera(cam_num).vid_res(setting, 3) + "p");
        
                %Overall Plot
                cluster_size = 10;
                figure();
                hold on;
                axis equal;
                [X,Y] = meshgrid(px(1:cluster_size:end),py(1:cluster_size:end)); %Position from pixel
                X = X(:);
                Y = Y(:);
                [Xe, Ye] = meshgrid(pe_x(1:cluster_size:end), pe_y(1:cluster_size:end)); %Res error from pixel
                E = sqrt(Xe.^2 + Ye.^2); %Total res error
        
                scatter3(X,Y, zeros(size(X)), 10, E(:), 'filled');
                scatter3(0,0,fb, 50, 'k', 'filled');
                plot3([X(1),0,X(1),X(end),0,X(end)], [Y(1), 0, Y(end),Y(end), 0, Y(1)], [0,fb,0,0,fb,0], 'k');
                title(camera(cam_num).name + " at " + fb + "ft - " + camera(cam_num).vid_res(setting, 1) + "x" + camera(cam_num).vid_res(setting, 2) + " " + camera(cam_num).vid_res(setting, 3) + "p");
                xlabel("ft");
                ylabel("ft");
                zlabel("ft");
                colorbar;
                clim(err_lim);
                colormap(c);
                hold off;
            end
        end
    end
end





