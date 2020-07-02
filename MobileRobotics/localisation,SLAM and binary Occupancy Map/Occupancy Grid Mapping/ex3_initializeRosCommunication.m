%% rossubscribers and rospublishers, scanner
rosshutdown
rosinit('localhost')
% subscribe
gazeboSub = rossubscriber('/gazebo/model_states');
odom_callback = rossubscriber('/odom', @odomCallback);
odom_callback2 = rossubscriber('/odom', @odomVelocityCallback);
camera_callback = rossubscriber('/camera/rgb/image_raw', @cameraCallback);
camera_callback2 = rossubscriber('/camera/depth/image_raw', ...
                                                         @cameraCallback2);
laserscan_callback = rossubscriber('/scan', @laserscanCallback);
%% callback functions
function odomCallback(~, msg)
% ODOMCALLBACK Subscriber callback function for pose data    
%   ODOMCALLBACK(~,MSG) returns no arguments - it instead sets a 
%   global variable to the values of position and orientation that are
%   received in the ROS message MSG.

    % declare global variable to store position and orientation
    global odomPose
    msgPose = msg.Pose.Pose;
    odomPose = xyTheta(msgPose);
end

function odomVelocityCallback(~, msg)

    % declare global variable to store linear and angular velocity
    global odomTwist
    msgTwist = msg.Twist.Twist;
    twist2D = vxvyOmega(msgTwist);
    odomTwist = [twist2D(1)
                 twist2D(3)];
end

function cameraCallback(~,msg)
    global cameraMsg;
    cameraMsg = msg;
end
function cameraCallback2(~,msg)
    global cameraMsg2;
    cameraMsg2 = msg;
end

function laserscanCallback(~,msg)
    global cartesian_reading;
    cartesian_reading = readCartesian(msg);
    global laser_msg;
    laser_msg = msg;
end

%% other functions

function pose2D = xyTheta(msgPose)
    % extract position and orientation from the ROS message and assign the
    % data to the returned variable.
    euler = quat2eul([msgPose.Orientation.W, ...
                      msgPose.Orientation.X, ...
                      msgPose.Orientation.Y, ...
                      msgPose.Orientation.Z]);
    pose2D = [msgPose.Position.X
              msgPose.Position.Y
              euler(1)];
end

function twist2D = vxvyOmega(msgTwist)
    % extract velocities from the ROS message and assign the
    % data to the global variable.
    twist2D = [msgTwist.Linear.X
               msgTwist.Linear.Y
               msgTwist.Angular.Z];
end
