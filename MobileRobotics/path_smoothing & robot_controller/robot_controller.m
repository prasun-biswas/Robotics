function [ w , v] = robot_controller( traj_x, traj_x_dot,traj_y, traj_y_dot, pos_x, pos_y,theta_head , dt)

% to run this first run the path_smoothing so we can get the trajectory
% plan then run this [ w , v] = path_following( traj_x, traj_x_dot,traj_y, traj_y_dot, 2, 0.5,0.2 , dt)
% where pos_x, pos_y and theta_head is the current position in x and y of
% the robot and thea_head is the current head ( xc, yc and theta_c)
close_point_index = -1;
dist_temp = 100000;
vmax = 0.5;
% find the closest point in the path to make it the desired point
for i = 1: size(traj_x,1)
    dist = sqrt((traj_x(i)- pos_x)^2+(traj_y(i) - pos_y )^2);
    if dist < dist_temp
        dist_temp = dist;
        close_point_index = i;
    end
end

% calculate the v at desired point so choose it a vel for the output also
v = sqrt( traj_x_dot(close_point_index).^2 + traj_y_dot(close_point_index).^2);
% max constrain for v
if v > vmax 
    v = vmax;
end
if v < -vmax
    v = -vmax;
end
% avoid situation robot not move at all, so at a small vel
if v == 0
    v = 0.001;
end
%this step following the slide that i has attached with the file
t_hat = [   traj_x_dot(close_point_index)/v ; traj_y_dot(close_point_index)/v ];
n_hat = [ - traj_y_dot(close_point_index)/v ; traj_x_dot(close_point_index)/v ];
t_hat_next = [   traj_x_dot(close_point_index + 1)/v ; traj_y_dot(close_point_index + 1)/v ];
t_hat_dot = [t_hat_next(1)- t_hat(1) , t_hat_next(2)- t_hat(2)] / dt;
Cc = t_hat_dot * n_hat / v;

theta_tw = atan2(traj_y_dot(close_point_index), traj_x_dot(close_point_index));

% the transformation matrix from the tangent frame to the world frame with
% the angle theta_tw
R_TW = [ cos(theta_tw) sin(theta_tw) ; -sin(theta_tw) cos(theta_tw)];
% the error respect to the world frame
e = blkdiag(R_TW,1)*[ pos_x - traj_x(close_point_index); pos_y - traj_y(close_point_index); theta_head - atan2(traj_y_dot(close_point_index), traj_x_dot(close_point_index))];

s_dot = v * cos(e(3)) / (1 - e(2)*Cc);

% create a model
A = [ 0 v; 0 0];
B = [0; 1];
% assume that the w_e is more concerned than the y_dot_e for the Q matrix
Q =  [ 1 0; 0 10];
% the value of R can be change depend on how aggressive we want for the
% control input
R = 1;
K = lqr(A,B,Q,R);

% following the equation in the slide : w_1 = (-k2*y_e -k3*theta_e) *v
w_1 = dot(-K,[ e(2) e(3)]) *v;
w = w_1 + Cc*s_dot;
end