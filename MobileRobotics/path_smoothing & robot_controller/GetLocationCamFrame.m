function PosFinal = GetLocationCamFrame(K,measPixRadius,pX,pY)

%Reference Length of 0.15m for ball
%Use it to calculate length in pixels
X1 = [0.15;0;0];
S = K*X1;
%Length of Ball in pixels
unitLength = S(1);

%By similar triangles, calculate z 
%Measured ball size in pixels to Unit Ball size in pixels 
Z = unitLength/measPixRadius;

X2 = [pX;pY;1];
%Location for Unit Z
Pos =  inv(K)*X2;

%Final Relative Location 
PosFinal = Pos *Z;
end