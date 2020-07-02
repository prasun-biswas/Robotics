%Function to Find Centroid and Area of all three color balls
%Returns NaN if no Color ball is found 
%Returns Centroid in image Coordinate and area
%Minimum Area specified selects ball size in pixels
function [r,g,b]= findball(im,MinArea)

%Get R, G and B component from Image
imR = im(:,:,1);
imG = im(:,:,2);
imB = im(:,:,3);
%Get Binary images satisfying color
outB = imR<30 & imG<30 & imB>60;
outG = imR<30 & imG>60 & imB<30;
outR = imR>60 & imG<30 & imB<30;
%Get Region Properties from Binary images
sR = regionprops(outR);
sB = regionprops(outB);
sG = regionprops(outG);

%Initialize Return Matrix to NaN
r = [NaN NaN NaN];
g = [NaN NaN NaN];
b = [NaN NaN NaN];

%Red Ball

%Finding Region with Largest Area 
iR = 0;
aR = 0;
%Loop through all regions and select one with largest area
for i= 1:size(sR,1)
    if sR(i).Area>aR && sR(i).Area>MinArea
        iR = i;
        aR = sR(i).Area;
    end
end
%If something is selected, add area and centroid
if iR>0
    r(1)= max(sR(iR).BoundingBox(3:4));
    r(2)= sR(iR).Centroid(1);
    r(3)= sR(iR).Centroid(2);
end


%Blue Ball

%Finding Region with Largest Area 
iB = 0;
aB = 0;
%Loop through all regions and select one with largest area
for i= 1:size(sB,1)
    if sB(i).Area>aB && sB(i).Area>MinArea
        iB = i;
        aB = sB(i).Area;
    end
end
%If something is selected, add area and centroid
if iB>0
    b(1)= max(sB(iB).BoundingBox(3:4));
    b(2)= sB(iB).Centroid(1);
    b(3)= sB(iB).Centroid(2);
end


%Green Ball

%Finding Region with Largest Area 
iG = 0;
aG = 0;
%Loop through all regions and select one with largest area
for i= 1:size(sG,1)
    if sG(i).Area>aG && sG(i).Area>MinArea
        iG = i;
        aG = sG(i).Area;
    end
end
%If something is selected, add area and centroid
if iG>0
    g(1)= max(sG(iG).BoundingBox(3:4));
    g(2)= sG(iG).Centroid(1);
    g(3)= sG(iG).Centroid(2);
end


end
