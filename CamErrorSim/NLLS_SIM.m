clc; close all; clear;

%Select Camera to analyze in this sim
cam_num = 1;

% Simulation parameters
fps = [1, 2, 5, 10, 14, 30];
color = ['b', 'm', 'r','k', 'k', 'k', 'g', 'k'];
sim_duration = 10; %s
sims = 25;
cors_acc = 2*3.28084; %Accuracy needed for coarse localization (ft)
fine_acc = 1*3.28084; %Accuracy needed for fine localization (ft)
fail_chance = 0.25; %Chance of not detecting the rgv while in frame

trial = "Trial " + (1:sims);
legends = strings(sims*length(fps), 1);

% RGV controllers


% Equations of motion (Ambiguous to the vehicle)
forwardEOM = @(obj, X, t) [X(1) + obj.max_speed(1)*t; X(2) + obj.max_speed(2)*t; X(3) + obj.max_speed(3)*t];
stillEOM = @(obj, X, t) [X(1)*ones(size(t)); X(2)*ones(size(t)); X(3)*ones(size(t))];
hoverEOM = @(obj, X, t) [X(1)*ones(size(t)); X(2)*ones(size(t)); X(3)*ones(size(t))];
circleEOM = @(obj, X, t) [X(1) + obj.radius*sin(t); X(2) + obj.radius*cos(t); X(3)*ones(size(t))];
sinEOM = @(obj, X, t) [X(1) + obj.radius*sin(t)+ obj.max_speed*t; X(2) + obj.max_speed*t; X(3)*ones(size(t))];
cosEOM = @(obj, X, t) [X(1)+ obj.max_speed*t; X(2) + obj.max_speed*t  + obj.radius*cos(t); X(3)*ones(size(t))];
EOMList = {forwardEOM, hoverEOM, circleEOM}; %, sinEOM, cosEOM};

%Measurement errors
noise_range = 12; %ft

% Simulate
% f_sim = figure();
f4 = figure();
grid on;

f_bar = figure();
pos = [100, 100, 900, 400];
f_bar.Position = pos;

err_log = -1*ones(sims * length(fps), 1); %Minimum error for each trail
cors_log = -1*ones(sims * length(fps), 1); %Time to reach coarse localization accuracy
fine_log = -1*ones(sims * length(fps), 1); %Time to reach fine localization accuracy
fps_log = -1*ones(sims * length(fps), 1); %Each trial's fps
counter = 1;

for fps_i = 1:length(fps)

    %Set up the timing
    steps = fps(fps_i)*sim_duration;
    dt = 1 / fps(fps_i);
    t = linspace(0,sim_duration, steps);

    %Adjust legends
    legends(counter) = fps(fps_i) + " FPS";

    for k = 1:sims
        clc;
        fprintf("Starting trial %i, %i FPS...\n", k, fps(fps_i));
        l = fprintf("Progress:\n");
        %This script will compare the errors for different video resolutions and
        %focal lengths
        drone = drone_params();
        
        %Define the equations of motion for the RGV
        rgv = rgv_params();
    
        mes_counter = 1; %Measurement counter
        rgvMeasurements = zeros(3,steps);
        droneMeasurements = zeros(3,steps);
        y_k = [];
        t_mes = []; %measured timesteps
        delete log;
        
        %Calculate path and set initial conditions
        randEOM = @() round(ceil(rand()*length(EOMList)));
        %rgvEOM = EOMList{randEOM()};
        rgvEOM = stillEOM;
        droneEOM = EOMList{randEOM()};
        %droneEOM = circleEOM;
        
        rgvPath = rgvEOM(rgv, rgv.X, t);
        dronePath = droneEOM(drone, drone.X, t);
    
        %Set the fps
        fps_log(counter) = fps(fps_i);
    
        %f1 = figure();
%         figure(f_sim);
        for i = 1:steps
            fprintf(repmat('\b',1,l));
            l = fprintf("Progress: %0.1f%%\n", (i/steps)*100);
    
            %Save the state for each timestep
            drone.X(1:3) = dronePath(:,i);
            rgv.X = rgvPath(:,i);
        
            %Simulate noise in a measurement
            %r = (2*noise_range.*rand(2,1)) - noise_range;
            %rgvMeasurements(:,i) = rgv.X + [r;0];
            drone = plotCam(drone, cam_num);
            
            [rgvTemp, Xblob] = locate(drone, rgv, drone.camera(cam_num), 1);
           
        
            %Plot for each step
%              plot3(rgvPath(1,1:i), rgvPath(2,1:i), rgvPath(3,1:i), 'LineWidth', 3);
%              hold on;
%              axis equal;
%              grid on;
        
            %Simulate errors in detection
            detect = (rand()>fail_chance);

            %Check that the drone is in camera view
            if ~isnan(rgvTemp) & detect
               droneMeasurements(:, mes_counter) = drone.X(1:3);
               rgvMeasurements(:,mes_counter) = rgvTemp' + [drone.X(1:2);0];
               y_k = [y_k;rgvMeasurements(:,mes_counter)];
               %rgvP = scatter3(Xblob(1)*(drone.camera(cam_num).proj_x(2) - drone.camera(cam_num).proj_x(1)) + drone.camera(cam_num).proj_x(1), Xblob(2)*(drone.camera(cam_num).proj_y(2) - drone.camera(cam_num).proj_y(1)) + drone.camera(cam_num).proj_y(1), 0, 100, 'red');
               t_mes = [t_mes, t(i)];
        
               %Calculate the non-linear least squares for cureve fitting
               [R_pred, ~] = NLS(y_k, rgvEOM, droneEOM, rgvMeasurements(:,1), drone.X0, t_mes);
               log(k).err(mes_counter) = norm(R_pred-rgv.X0);

               %Check if accuracy demands were met
               if cors_log(counter) == -1 && log(k).err(mes_counter) < cors_acc
                   cors_log(counter) = t(i);
               end

               if fine_log(counter) == -1 && log(k).err(mes_counter) < fine_acc
                   fine_log(counter) = t(i);
               end
        
               %Update the measurement counter
               mes_counter = mes_counter +1;
            end
        
            %Finish plotting
%             plot3(dronePath(1,1:i), dronePath(2,1:i), dronePath(3,1:i), 'LineWidth', 3);
%             scatter3(rgvMeasurements(1,1:mes_counter-1), rgvMeasurements(2,1:mes_counter-1), rgvMeasurements(3,1:mes_counter-1), 10, 'filled');
%             drone = plotCam(drone, cam_num);
%             title(trial(k));
%             legend(["", "RGV Actual Location", "Drone Path", "Localization Measurements"]);
%             hold off;
%         
%             drawnow;
        %     pause(0.2);
        end
    
        %delete y_k
        
%         %Calculate the non-linear least squares for cureve fitting
%         [R_pred, path] = NLS(y_k, rgvEOM, droneEOM, rgvMeasurements(:,1), drone.X0, t_mes);
%         rgvPredPath = rgvEOM(rgv, R_pred, t) - droneEOM(drone, drone.X0, t);
%         rgvPredInerPath = rgvEOM(rgv, R_pred, t);
%         
%         %Display the error in ft
%         fprintf("X error: %0.2f ft\n", abs(R_pred(1)-rgv.X0(1)));
%         fprintf("Y error: %0.2f ft\n", abs(R_pred(2)-rgv.X0(2)));
%         fprintf("Z error: %0.2f ft\n", abs(R_pred(3)-rgv.X0(3)));
%         fprintf("Total Error: %0.2f ft\n", norm(R_pred-rgv.X0));
%         
%         %PLace the predicted path on the final sim
%     %     figure(f1);
%     %     hold on;
%     %     plot3(rgvPredInerPath(1,:), rgvPredInerPath(2,:), rgvPredInerPath(3,:), 'k', 'LineWidth',4);
%     %     hold off;
%         
%         
%         %Plot the relative RGV positons relative to the drone
%         figure();
%         hold on;
%         axis equal;
%         grid on;
%         scatter(rgvMeasurements(1,:) - droneMeasurements(1,:), rgvMeasurements(2,:) - droneMeasurements(2,:), 'filled');
%         %scatter(path(1,:) - dronePath(1,1), path(2,:) - dronePath(2,1), 'kx');
%         plot(rgvPath(1,:) - dronePath(1,:), rgvPath(2,:) - dronePath(2,:), 'LineWidth', 3);
%         plot(rgvPredPath(1,:), rgvPredPath(2,:), 'k', 'LineWidth', 2);
%         legend(["Measured Pos", "True Path", "Predicted Path"]);
%         xlabel("X (ft)");
%         ylabel("Y (ft)");
%         title("Relative Measurements for Trial " + k);
%         hold off;
%         
%         %Plot the RGV positons in the inertial frame
%         figure();
%         hold on;
%         axis equal;
%         grid on;
%         scatter(rgvMeasurements(1,:), rgvMeasurements(2,:), 'filled');
%         %scatter(path(1,:), path(2,:), 'kx');
%         plot(rgvPath(1,:), rgvPath(2,:), 'LineWidth', 3);
%         plot(rgvPredInerPath(1,:), rgvPredInerPath(2,:), 'k', 'LineWidth', 2);
%         legend(["Measured Pos", "True Path", "Predicted Path"]);
%         xlabel("X (ft)");
%         ylabel("Y (ft)");
%         title("Inertial Measurements for Trial " + k);
%         hold off;

        %Save the best error case
        log(k).t_mes = t_mes; %Save the measurement times
        try
            err_log(counter) = min(log(k).err);
        catch
            err_log(counter) = cors_acc + 1;
        end
        
        %Plot the error over time
        figure(f4);
        hold on;
        if ~isempty(t_mes)
            if length(t_mes) == length(log(k).err), plot(t_mes, log(k).err, color(fps_i), 'LineWidth', 2); end
        end
        hold off;
    
        %Update the box plot
        figure(f_bar)
        clf;
        tl = tiledlayout(1,2,'TileSpacing','Compact');
        title(tl, "Non-Linear Least Squares Benchmarks with " + (fail_chance*100) + "% Detection Failure");
        nexttile
        hold on;
        grid on;
        yline(cors_acc, '--', "Coarse", 'LabelVerticalAlignment', 'middle');
        yline(fine_acc, '--',  "Fine", 'LabelVerticalAlignment', 'middle');
        boxplot(err_log(1:counter), fps_log(1:counter))
        title("Lowest Estimate Error");
        xlabel("FPS");
        ylabel("Error (ft)");
        hold off;

        nexttile
        hold on;
        grid on;
        %boxplot(cors_log(1:counter), fps_log(1:counter));
        for o = 1:fps_i
           %define set of data per fps
           c = cors_log((sims)*(o-1) + (1:sims));
           f = fine_log((sims)*(o-1) + (1:sims));

           %Get averages
           cors_avgs(o) = mean(c(c>=0));
           fine_avgs(o) = mean(f(f>=0));

           %Format strings
           if fps(o) == 14 || fps(o) == 30
               cs = sprintf("%0.0f ms", cors_avgs(o)*1000);
               fs = sprintf("%0.0f ms", fine_avgs(o)*1000);
    
               %Plot the values
               if cors_avgs(o) ~= fine_avgs(o)
                   text(fps(o)+0.02, cors_avgs(o)-0.05, cs);
                   text(fps(o)+0.02, fine_avgs(o)+0.05, fs);
               else
                   text(fps(o)+0.02, fine_avgs(o)+0.05, fs);
               end
           end
        end
        plot(fps(1:fps_i), cors_avgs, 'o-', 'LineWidth', 2);
        plot(fps(1:fps_i), fine_avgs, 'o-', 'LineWidth', 2);
        xline(14, '--', "4K FPS", "LabelHorizontalAlignment","left", "LabelVerticalAlignment","middle");
        xline(30, '--', "2K FPS", "LabelHorizontalAlignment","left", "LabelVerticalAlignment","middle");
        title("Average Time to Meet Requirement Accuracy");
        xlabel("FPS");
        ylabel("Time to Reach (s)");
        ylim([-0.1, 1]);
        xlim([0, fps(end)+2]);
        legend(["Course Requirement", "Fine Requirement"]);
        xticks(fps)
        hold off;

%         nexttile
%         hold on;
%         grid on;
%         plot(fps(1:fps_i), cors_avgs, 'o-', 'LineWidth', 2);
%         plot(fps(1:fps_i), fine_avgs, 'o-', 'LineWidth', 2);
%         xline(14, '--', "4K FPS", "LabelHorizontalAlignment","left", "LabelVerticalAlignment","middle");
%         xline(30, '--', "2K FPS", "LabelHorizontalAlignment","left", "LabelVerticalAlignment","middle");
%         yline(60, '--', "Coarse", 'LabelVerticalAlignment', 'middle', 'Color', [0 0.4470 0.7410]);
%         yline(30, '--',  "Fine", 'LabelVerticalAlignment', 'middle', 'Color', [0.8500 0.3250 0.0980]);
%         title("Average Time to Meet Requirement Accuracy");
%         xlabel("FPS");
%         ylabel("Time to Reach (s)");
%         ylim([-0.1, 62]);
%         xlim([0, fps(end)+2]);
%         legend(["Course Requirement", "Fine Requirement"]);
%         xticks(fps)
%         hold off;
% 
%         nexttile
%         hold on;
%         grid on;
%         %boxplot(fine_log(1:counter), fps_log(1:counter))
%         plot(fps(1:fps_i), fine_avgs, 'LineWidth', 2);
%         title("Fine Requirement");
%         xlabel("FPS");
%         ylabel("Time to Reach (s)");
%         hold off;
         drawnow;

        counter = counter+1;
    end
end

figure(f4);
hold on;
title("NLS error");
ylabel("Initial Prediction Error (ft)");
xlabel("Time (s)");
legend(legends);
hold off;

saveas(f_bar, "Figures/NLS_Bench_RTK=" + drone.use_rtk + "_SIMS=" + sims + "_T=" + sim_duration + "_FC=" + fail_chance*100 + "_2.jpg");



