%Subscribe to Turtle Bot
%ipaddress = '192.168.1.129';
rosinit(ipaddress);
%Subscribe to Camera Message
camSub = rossubscriber('/camera/rgb/image_raw/compressed');
%Subscribe to Camera Info
camInfoSub = rossubscriber('/camera/rgb/camera_info');
%Receive Camera info
camInfo = receive(camInfoSub,3);
%Find Intrinsic K 
K = reshape(camInfo.K,[3,3])';
%Continue Receiving until force exit
i = 1
while i==1
    %Receive Video frame
    camMsg = receive(camSub,3);
    %Read Camera image
    im = readImage(camMsg);
    %Find color ball
    [r,g,b] = findball(im,5);
    %Find location of Red Ball from region properties
    if ~isnan(r(1))
        redBall   = GetLocationCamFrame(K,r(1),r(2),r(3))
    end
    %Find location of Blue Ball from region properties
    if ~isnan(b(1))
        blueBall   = GetLocationCamFrame(K,b(1),b(2),b(3))
    end
    %Find location of Green Ball from region properties
    if ~isnan(g(1))
        greenBall   = GetLocationCamFrame(K,g(1),g(2),g(3))
    end
    imshow(im);
end
rosshutdown