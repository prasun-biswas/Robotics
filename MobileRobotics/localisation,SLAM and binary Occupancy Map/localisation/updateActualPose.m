function actualPose = updateActualPose(gazeboSub)
% input: gazeboSub = rossubscriber('/gazebo/model_states');

    msg = gazeboSub.LatestMessage;
    % msg.Name() % list of everything in the environment, 
    %            % robot is "mobile_base"
    
    msgPose = msg.Pose(strcmp(msg.Name, 'mobile_base'));
    actualPose = xyTheta(msgPose);
end

%% subfunctions

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
