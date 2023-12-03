% Non Linear Least Squares for RGV Localization from a Drone
% Author: Carson Kohlbrenner
% Date 11/13/23

%Inputs:

%Outputs:

function [R_pred, path] = NLS(y_k, eom_rgv, eom_dr, R0, D0, t)
    drone = drone_params();
    rgv = rgv_params();

    %Expected model for measurements
%     h_x = @(R0) getCol(eom_rgv(rgv, R0, t)' - eom_dr(drone, D0, t)',1);
%     h_y = @(R0) getCol(eom_rgv(rgv, R0, t)' - eom_dr(drone, D0, t)',2);
%     h_z = @(R0) getCol(eom_rgv(rgv, R0, t)' - eom_dr(drone, D0, t)',3);
    h_x = @(R0) getCol(eom_rgv(rgv, R0, t)',1);
    h_y = @(R0) getCol(eom_rgv(rgv, R0, t)',2);
    h_z = @(R0) getCol(eom_rgv(rgv, R0, t)',3);
    h_k = @(R0) concatCol([h_x(R0), h_y(R0), h_z(R0)]');
    %Covariance matrix
    sig_r = 0.1; % mm, Guess of the standard deviation of error?
    R_i = sig_r^2;
    R_big = eye(length(y_k)) * R_i;

    %Cholesky Decomposition
    A = chol(inv(R_big));
    y_a = A*y_k;
    h_a = @(h_k) A*h_k;

    %Calculate the Jacobian
    alpha = 0.01; %Step size for calculating the jacobian
    Alpha = eye(3)*alpha;

    H = @(R0) (alpha^-1)*[h_a(h_k(R0+Alpha(:,1))) - h_a(h_k(R0)), ...
                          h_a(h_k(R0+Alpha(:,2))) - h_a(h_k(R0)), ...
                          h_a(h_k(R0+Alpha(:,3))) - h_a(h_k(R0))];
    
     %Gauss-Newton Method to Solve
    iter = 0;
    max_iter = 1000; %Number of iterations to terminate on if solution not found
    epsilon = 0.001; %Termination error 
    R_pred = R0;
    path = R0;
    beta = 0.01;
    while iter < max_iter

        %calculate the jacobian and get the change in position guess
        Jac = H(R_pred);
        ds = inv(Jac' * Jac)*Jac' * (y_a - h_a(h_k(R_pred)));
        R_pred = R_pred + beta*ds;
        path = [path,R_pred];

        %Termination Condition
        if norm(beta*ds) < epsilon
            %disp("Predicted Location: " + R_pred)
            return
        end

        if isnan(ds)
            %disp("Did not converge!");
            R_pred = [0;0;0];
            return
        end

        iter = iter+1;
    end

    %disp("Did not converge!");
    return

end
