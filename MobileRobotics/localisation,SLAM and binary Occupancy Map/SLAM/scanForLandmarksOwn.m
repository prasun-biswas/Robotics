
function [angles, ranges] = scanForLandmarksOwn(cameraImage, depthImage)
    landmarkRadius = 0.15;
    fx = 554.254691191187; % focal length
    fy = 554.254691191187;
    halfWidthX = (320.5 - 1); % 0.5 * image width, (cx - 1)
    halfWidthY = (240.5 - 1);
    maxHueError = pi/48; % when hue is between [0 2pi]
    minS = 0.95;
    minV = 0.3;

    imageSize = size(cameraImage);
    imageWidthXInPixels = imageSize(2);
    imageXCoordinates = linspace(-halfWidthX, ...
                                  halfWidthX, imageWidthXInPixels);
    imageAnglesH = atan(-imageXCoordinates./fx);
    
    imageWidthYInPixels = imageSize(1);
    imageYCoordinates = linspace(-halfWidthY, ...
                                  halfWidthY, imageWidthYInPixels);
    imageAnglesV = atan(-imageYCoordinates./fy);
    
    imageHSV = rgb2hsv(cameraImage);
    sufficientSV = and(imageHSV(:,:,2) > minS, ...
                       imageHSV(:,:,3) > minV);
    angles = NaN(3,1);
    ranges = NaN(3,1);
    for lmColor = 0:2 % 0 red, 1 green, 2 blue
        
        hueIsCorrect = maxHueError > ...
            abs(angleDifference((2*pi).*imageHSV(:,:,1), 2*pi*lmColor/3));
        landmarkPixels = and(hueIsCorrect, sufficientSV);
        if sum(landmarkPixels) < 32
            continue
        end
        
%         [~, columnIndices] = find(landmarkPixels);
        [rowIndices, columnIndices] = find(landmarkPixels);
        pixelBoundsH = [min(columnIndices) max(columnIndices)];
        pixelBoundsV = [min(rowIndices)    max(rowIndices)];
        if any(pixelBoundsH == [1 imageWidthXInPixels]) % landmark is only
            continue                     % partially within sensor range
        end
        
        boundAnglesH = sort(imageAnglesH(pixelBoundsH));
        boundAnglesV = sort(imageAnglesV(pixelBoundsV));
%         centroidAngleH = imageAnglesH(median(columnIndices));
%         angles(lmColor+1) = mean([mean(boundAnglesH); centroidAngleH]);
        angles(lmColor+1) = mean(boundAnglesH);
        ranges(lmColor+1) = mean( ...
            [landmarkRadius / sin(diff(boundAnglesH)/2);
             landmarkRadius / sin(diff(boundAnglesV)/2)]);
        if any(pixelBoundsV == [1 imageWidthYInPixels]) % vertically only
                                                        % partially seen
            ranges(lmColor+1) = landmarkRadius / sin(diff(boundAnglesH)/2);
        end
        
        
        if ranges(lmColor+1) > 5 % depth image has maximum depth = 5
            continue
        end
        
        if exist('depthImage', 'var')
            depthPixels = depthImage(landmarkPixels);
            validDepthPixels = depthPixels(isfinite(depthPixels));
            ranges(lmColor+1) = median(validDepthPixels(:)) ...
                + (2/3)*landmarkRadius;
        end
    end
end

%% subfunctions

% similar to matlab function "angdiff", but supports
% inputs where other input is array and the other a single value
function angleDifferences = ...
    angleDifference(oldDirections, newDirections)

    sizeNew = size(newDirections);
    sizeOld = size(oldDirections);
    if all(sizeNew == 1)
        newDirections = newDirections + zeros(sizeOld);
    elseif all(sizeOld == 1)
        oldDirections = oldDirections + zeros(sizeNew);
    end
    angleDifferences = angdiff(oldDirections, newDirections);
    return
end
