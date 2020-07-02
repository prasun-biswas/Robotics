%% ex-2a 
% (for visualisation of kalman filter run gazebo with moro_office.world and move the turtlebot)
% using turtlebot keyboard teleop see kalman results.png for comparative evaluation
% the green '*' are from odometry data blue 'o' are from kalman results when landmark is visible
% the red 'o' are from kalman result when landmark isnot visible.

% reconnect with ros
rosshutdown
rosinit(ipaddress)

% load "moroparams.mat" and show the map
figure
load('moroparams.mat')
hold
show(map)

% initialise scanner object and define landmark
scanner = MoroLandmarkScanner();
landmark = [[5,5];[1,2.5];[-5.5,0]];

% initialise postion and orientation
x_filter = 0; %initialising at 0
y_filter = 0; %initialising at 0
theta_filter = 0; %initialising at 0
theta_filter_pred = 0; %initialising at 0
landmark_x_actual = 0; %initialising at 0
landmark_y_actual = 0; %initialising at 0

% initialise frame transform
tftree = rostf;
pause(1);

% initialise refresh rate, linear velocity, angular velocity
dt_filter = 0.1;
v_filter = 0.2;
w_filter = 1;

% initialise  StateCovariance and measurement Covariance
initial_stateCovariance = eps*eye(3);
initial_measurementCovariance = eps*eye(2);
next_mean = [0;0;0];
counter = 0;

while 1
    counter = counter + 1;
    % take stateCovariance as the initial value for first step
    if (counter == 1)
        stateCovariance = initial_stateCovariance;
    else
        stateCovariance = next_stateCovariance;
    end
    
    [A,R] = scanner.scanForLandmarks(); % angle,range reading from landmark scanner object
    [x_cord,y_cord] = pol2cart(A,R);  % converting to cartesian coordinates
    x_pt_index = find(~isnan(x_cord)); % get the index of landmark
    y_pt_index = find(~isnan(y_cord)); % get the index of landmark
    
    odom = rossubscriber('/odom'); % subscribe to odometry data of turtlebot
    odomdata = receive(odom,3); % receive odometry data
    vel = odomdata.Twist.Twist; % get velocity components
    v_filter = vel.Linear.X; % linear velocity
    w_filter = vel.Angular.Z; % angular velocity
    quat = odomdata.Pose.Pose.Orientation; % get quaternion
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
    theta_filter = angles(1); % get orientation angle
    
    % plot x, y from odom data
    x = odomdata.Pose.Pose.Position.X; 
    y = odomdata.Pose.Pose.Position.Y;
    plot(x, y, 'g--*')
    
    % if landmark is visible locate w.r.t to the landmark
    if(~isnan(x_pt_index))
        % compute x,y components from landmark measurement
        X_turtlebot = landmark(x_pt_index,1) - R(x_pt_index)*cos(theta_filter + A(x_pt_index)); %x_cord(x_pt_index);
        Y_turtlebot = landmark(y_pt_index,2) - R(y_pt_index)*sin(theta_filter + A(y_pt_index)); %y_cord(y_pt_index);
        x_filter = X_turtlebot;
        y_filter = Y_turtlebot; 
        
        % get actual landmark location
        landmark_x_actual = landmark(x_pt_index,1);
        landmark_y_actual = landmark(y_pt_index,2);
        
        % call ekf function ie the Kalman Filter algorithm
        [next_mean,next_stateCovariance] = ekf(x_filter, y_filter, theta_filter, dt_filter, v_filter, w_filter, landmark_x_actual, landmark_y_actual, stateCovariance, initial_measurementCovariance);
        plot(next_mean(1), next_mean(2), 'b--o')

    else
        % approxiamte location based on previous location and velocity. 
        disp('no landmark visible, taking previous states as initial')
        counter = 0;
        x_filter_pred = next_mean(1) + (-v_filter/w_filter*sin(theta_filter)+v_filter/w_filter*sin(theta_filter+w_filter*dt_filter)); %v_filter*cos(theta_filter_pred)*0.1; %
        y_filter_pred = next_mean(2) + v_filter/w_filter*cos(theta_filter)-v_filter/w_filter*cos(theta_filter+w_filter*dt_filter); %v_filter*sin(theta_filter_pred)*0.1; %
        [next_mean,next_stateCovariance] = ekf(x_filter_pred, y_filter_pred, theta_filter, dt_filter, v_filter, w_filter, landmark_x_actual, landmark_y_actual, stateCovariance, initial_measurementCovariance);
        plot(next_mean(1), next_mean(2), 'r--o')
    end
    pause(0.1)
end 