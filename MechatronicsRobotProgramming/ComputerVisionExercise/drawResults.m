function [img_rgb]=drawResults(img_rgb,data)

%% Adds face bounding box and detected points into given image (note: image is returned, not displayed)

% Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
% format required by insertShape.
bboxPolygon = reshape(data.bboxPoints', 1, []);
xyPoints = data.oldPoints;

% Display a bounding box around the detected face.
img_rgb = insertShape(img_rgb, 'Polygon', bboxPolygon, 'LineWidth', 3);

% Display detected corners.
img_rgb = insertMarker(img_rgb, xyPoints, '+', 'Color', 'white');