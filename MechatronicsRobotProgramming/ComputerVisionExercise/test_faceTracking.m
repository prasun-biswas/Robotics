%% This script is used to test the function 'face_detect.m'

% Create a global video player 
global videoPlayer
videoPlayer = vision.VideoPlayer('Position', [500 250 680 500]);

% Create a ROS image message to test our callback function
msg = rosmessage('sensor_msgs/Image');
msg.Encoding = 'rgb8';

% Create a video reader object
reader = VideoReader('visionface.avi');

% Loop through all the frames
while hasFrame(reader) 
    % Read next frame
    frame = readFrame(reader);
    % Resize
    frame = imresize(frame, 0.8);
    % Write the current frame into our ROS message
    writeImage(msg, frame)
    % Execute your function
    %face_detect(msg, msg)
    face_solution(msg, msg)
    
    % Break if video player is closed
    if ~isOpen(videoPlayer)
        break
    end
    % Pause to make the video clip last longer
    pause(0.005)
end

