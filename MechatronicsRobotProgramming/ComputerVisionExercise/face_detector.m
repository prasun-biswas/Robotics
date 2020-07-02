function [data] = face_detector(img_gray,data)

%% Run face detector for single frame and initialise feature points if face is detected.
% inputs: 'img_gray' is input image in gray-scale format. 
%         'data' is a data structure containing current status (see exercise
%              material for details).

% Apply detector
bbox = data.faceDetector.step(img_gray);

% Check if anything was found and return if not
if isempty(bbox)
    return;
end

% Detect feature points inside the face region. (Needed for tracking). 
points = detectMinEigenFeatures(img_gray, 'ROI', bbox(1, :));

% Re-initialize the point tracker to the newly found feature points. 
xyPoints = points.Location;
data.nPts = size(xyPoints,1);
release(data.pointTracker);
initialize(data.pointTracker, xyPoints, img_gray);

% Save a copy of the points.
data.oldPoints = xyPoints;

% Convert the rectangle represented as [x, y, w, h] into an
% M-by-2 matrix of [x,y] coordinates of the four corners. This
% is needed to be able to transform the bounding box to display
% the orientation of the face.
data.bboxPoints = bbox2points(bbox(1, :));

