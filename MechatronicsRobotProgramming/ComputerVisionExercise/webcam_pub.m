function webcam_pub(pub)
% This function is used to publish webcam feed to a ROS-network
    global videoPlayer

    % Create a ROS image message to test our callback function
    msg = rosmessage('sensor_msgs/Image');
    msg.Encoding = 'rgb8';

    % Create a webcam instance
    cam = webcam();
    frame_count = 0;  
    
    % Read webcam for 500 frames or until the player is closed
    while frame_count < 500
        % Get a frame
        frame = snapshot(cam);
        % Write the current frame into our ROS message
        writeImage(msg, frame)
        % Publish the image
        send(pub,msg);
        frame_count = frame_count + 1;
        % Break if video player is closed
        if ~isOpen(videoPlayer) && frame_count > 10
            break
        end
    end

end
