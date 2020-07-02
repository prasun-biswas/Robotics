%% ex-2b
%% load waypoints and straight line joining them
map = load('moroparams.mat');
hold on
show(map.map)
path_to_goal = map.path;

%% initialise various components to compute path smoothing

num_segments = size(path_to_goal,1)-1; % number of curves joining way-points

weight = 1; % weight factor to tune how close to go with the way-points

t = linspace(0,10,101); % take 101 steps to reach from one point to another
t = transpose(t); 
dt = t(2) -t(1); % time difference between two steps
path_x_desired = []; % desired path x component 
path_y_desired = []; % desired path y component

% compute the step points to reach from consecutive way-points
for pts = 1:num_segments
    % velocity to be taken between two consecutive way-points
    vel_seg = sqrt(((path_to_goal(pts+1,1) - path_to_goal(pts,1)) / t(end)) ^ 2 + ((path_to_goal(pts+1,2) - path_to_goal(pts,2)) / t(end)) ^ 2);
    % orientation to be taken between two consecutive way-points
    theta_seg = atan2(path_to_goal(pts+1,2) - path_to_goal(pts,2), path_to_goal(pts+1,1) - path_to_goal(pts,1));
    % x and y component of each step point
    xt_ = path_to_goal(pts,1) + vel_seg * cos(theta_seg) * t;  
    yt_ = path_to_goal(pts,2) + vel_seg * sin(theta_seg) * t;
    % stack the step points in verticle resulting in a vector of dimension
    % [number of segments*(number of step points ,here 101), 1]
    path_x_desired = [path_x_desired; xt_];
    path_y_desired = [path_y_desired; yt_];
end  
%% creation of polynomial matrix P
% initialising components of P matrix
Ad = [1, dt, 0.5*dt.^2;0, 1, dt;0, 0, 1]; 
Bd = [ 1/6*dt.^3, 0.5*dt.^2, dt];

% allocating memory for P matrix
P =  zeros(size(t,1) - 1,size(t,1) - 1);
Pdot = zeros(size(t,1) - 1,size(t,1) - 1);
Pddot = zeros(size(t,1) - 1,size(t,1) - 1);

Pint = zeros(size(t,1) ,3);
Pdotint = zeros(size(t,1),3);
Pddotint = zeros(size(t,1),3);

% allocating values to P matrix
for i = 1: size(t,1)-1
    for j = 1 : i
        temp = Ad^(i-j)*transpose(Bd);
        P(i,j) = temp(1);
        Pdot(i,j) = temp(2);
        Pddot(i,j) = temp(3);
    end
end

% allocating values to Pint matrix
for i = 1:size(t,1)
    temp = Ad^(i);
    Pint(i,:) = temp(1,:);
    Pdotint(i,:) = temp(2,:);
    Pddotint(i,:) = temp(3,:);
end

P = [ zeros(1, size(P,1)); P];
Pdot = [ zeros(1, size(Pdot,1)); Pdot];
Pddot = [ zeros(1, size(Pddot,1)); Pddot];

P = [ Pint P];
Pdot = [ Pdotint Pdot];
Pddot = [ Pddotint Pddot];
%% initialising start point and end point constraints

% initial x, y position
xt_initial = path_to_goal(1,1);
yt_initial = path_to_goal(1,2);

% initial x,y velocity
xt_dot_initial = 0;
yt_dot_initial = 0;

% initial x,y accelatation
xt_ddot_initial = 0;
yt_ddot_initial = 0;

% final x,y postion     
xt_finish = path_to_goal(end,1);
yt_finish = path_to_goal(end,2);

% final x,y velocity     
xt_dot_finish = 0;
yt_dot_finish = 0;

% final x,y accelaration
xt_ddot_finish = 0;
yt_ddot_finish = 0;

%% call cubic spline smoothing function to receive smoothed trajectory
[locationX_traj, velocityX_traj, accelarationX_traj] = cubicspline_smoothing(num_segments, xt_initial, xt_dot_initial, xt_ddot_initial, xt_finish, xt_dot_finish, xt_ddot_finish, P, Pdot, Pddot, path_x_desired, weight);
[locationY_traj, velocityY_traj, accelarationY_traj] = cubicspline_smoothing(num_segments, yt_initial, yt_dot_initial, yt_ddot_initial, yt_finish, yt_dot_finish, yt_ddot_finish, P, Pdot, Pddot, path_y_desired, weight);

% plot trajectory
plot(path_to_goal(:,1),  path_to_goal(:,2), 'o')
plot(locationX_traj, locationY_traj, '-b')
%% end of ex-2b
%% {ex-2c is called as a function from the code below}
%% ex-2d (first run gazebo simulator and then run the code below) 
%% call path_follower controller
rosshutdown
rosinit
while 1
    odom = rossubscriber('/odom'); % subscribe to odometry data of turtlebot
    odomdata = receive(odom,3); % receive odom data
    quat = odomdata.Pose.Pose.Orientation; % get quaternion
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = angles(1); % get orientation
    x = odomdata.Pose.Pose.Position.X; % get current position x
    y = odomdata.Pose.Pose.Position.Y; % get current position y
    % try catch to detect when the turtlebot reaches its goal point
    try
        % call robot controller function to receive linear velocity and 
        % angular velocity to be sent to turtle bot
        [w,v] = robot_controller(locationX_traj, velocityX_traj, locationY_traj, velocityY_traj, x, y, theta, dt);
        % publish in the following topic
        robot = rospublisher('/mobile_base/commands/velocity');
        % create velocity message
        velmsg = rosmessage(robot);
        velmsg.Angular.Z = w;
        velmsg.Linear.X = v;
        % send velocity message to turtlebot
        send(robot,velmsg);
    catch
        % break when the turtle bot reaches the end point
        break
    end
end

                                                                                            
                                                                                             

