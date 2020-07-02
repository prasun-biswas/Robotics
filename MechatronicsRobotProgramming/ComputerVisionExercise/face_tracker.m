function [data]=face_tracker(img_gray,data)

%% Run KLT feature point tracker for given frame and detected points. 
% inputs: 'img_gray' is input image in gray-scale format. 
%         'data' is a data structure containing current status (see exercise
%              material for details).

% Update tracker using new frame
[xyPoints, isFound] = step(data.pointTracker, img_gray); % Outputs the new locations of the points and list which point are still tracked succesfully.
% Read points the are succesfully tracked.
visiblePoints = xyPoints(isFound, :);
% Read previous location of the corresponding points (need to determine the global motion). 
oldInliers = data.oldPoints(isFound, :);

% Update the number of tracked points. 
data.nPts = size(visiblePoints, 1);

% If the number of succesfully tracked points is more than 10, determine
% the face location using the tracked points. Otherwise, consider that
% tracking is lost and new face must be detected.
if data.nPts >= 10
    
    % Estimate the geometric transformation (similarity = translation, rotation, and scaling) between the old and new
    % locations of the tracked points. We assume that this gives us the
    % motion of the face.
    [xform, ~, visiblePoints] = estimateGeometricTransform(...
        oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
    
    % Apply the obtained transformation to the face bounding box (it shoud move similarly).
    data.bboxPoints = transformPointsForward(xform, data.bboxPoints);
    
    % Save data
    data.oldPoints = visiblePoints;
    setPoints(data.pointTracker, data.oldPoints);
      
end