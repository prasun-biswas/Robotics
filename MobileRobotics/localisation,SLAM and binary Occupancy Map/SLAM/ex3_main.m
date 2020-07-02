
close all

% Run this script after setting correct IP addresses to the beginning of
% ex3_initializeRosCommunication.m.
% Alternatively, set 'simulateOutput' to 'true' in the input section below
% to perform a test without connecting to ROS.

%% input

% Covariance matrices for Kalman localization
R = [0.01 0
     0    0.2]; % measurement inaccuracy
M = [1e-3 0
     0    1e-3]; % control noise
Pinitial = eye(3).*1e-3; % pose covariance
Pinitial(3,3) = Pinitial(3,3) * (2*pi)^2;

% set limits (path smoothing and control velocities will obey)
velocityLimit = 0.2;
angularVelocityLimit = 2*pi * 0.2;
accelerationLimit = 0.2;
angularAccelerationLimit = 2*pi * 0.2;

% (this is for testing only)
% planned path can exceed limits up to limit times this (accelerations ^2)
% (control velocities still obey limits regardless of this setting)
allowedScaleToLimit = 1; % should be 1

% motion controller parameters
controlParams = 0.1.*[1 1 1];
simulateOutput = true;
 % below: these are active only if simulateOutput == false
 dataAssociationProblem = false; % this is checked first
  SLAM = true;
 useOdomVelocitiesAsObservedVelocities = false; % this is checked first
  useActualVelocitiesAsObservedVelocities = true;
 useDepthImages = false;
 landmarkPositions = [5 5; 1 2.5; -5.5 0];
 deltat = 0.5;
  adaptiveTimeStep = false; % don't use this
   iterationLimit = 10000; % only for adaptiveTimeStep == true


%% Extended Kalman initialization

syms x y theta dt v w mx my r alpha real

% motion model
% fSym = [x;y;theta] + [v*dt * cos(theta + w*dt/2)
%                       v*dt * sin(theta + w*dt/2)
%                       w*dt];
archPathLength = abs(w*dt);
linearPathLength = sqrt((cos(w*dt) - 1)^2 + sin(w*dt)^2);
archCorrection = (linearPathLength + 1e-6) / (archPathLength + 1e-6);
fSym = [x;y;theta] + [archCorrection * v*dt * cos(theta + w*dt/2)
                      archCorrection * v*dt * sin(theta + w*dt/2)
                      w*dt];
f = matlabFunction(fSym, 'vars', [x y theta v w dt]);
archCorrectionFun = matlabFunction(archCorrection, 'vars', [w dt]);

% Motion model Jacobian
FSym = jacobian(fSym,[x;y;theta]);
FFun = matlabFunction(FSym, 'vars', [x y theta v w dt]);

% Control model Jacobian
VSym = jacobian(f,[v;w]);
VFun = matlabFunction(VSym, 'vars', [x y theta v w dt]);

% Measurement model
hSym = [sqrt((mx - x)^2 + (my - y)^2)
        atan2(my - y, mx - x) - theta];
h = matlabFunction(hSym, 'vars', [x y theta mx my]);

% Measurement Jacobian
HSym = jacobian(h, [x;y;theta]);
HFun = matlabFunction(HSym, 'vars', [x y theta mx my]);

% Measurement Jacobian 2
H2Sym = jacobian(h, [mx;my]);
H2Fun = matlabFunction(H2Sym, 'vars', [x y theta mx my]);

% Inverse measurement model
gSym = [x + r*cos(theta + alpha)
        y + r*sin(theta + alpha)];
g = matlabFunction(gSym, 'vars', [x y theta r alpha]);

% Inverse measurement Jacobian
GSym = jacobian(g, [x;y;theta]);
GFun = matlabFunction(GSym, 'vars', [x y theta r alpha]);

% Inverse measurement Jacobian 2
G2Sym = jacobian(g, [r;alpha]);
G2Fun = matlabFunction(G2Sym, 'vars', [x y theta r alpha]);

clear fSym FSym VSym hSym HSym H2Sym gSym GSym G2Sym
clear x y theta dt v w mx my r alpha
clear linearPathLength archPathLength archCorrection

%%

load('moroparams.mat')

% A = cscvn(path.');
% 
% figure(2)
% show(map)
% hold on
% plot(path(:,1),path(:,2),'.-')
% fnplt(A)
% hold off

%% path smoothing


% time between path points is set to be relative to segment length
edgesComplex = diff(path(:,1)) + 1i.*diff(path(:,2));
translationCosts = abs(edgesComplex);
edgeCosts = translationCosts;

% time at each given point, total time scaled to 1
pathTime = [0; cumsum(edgeCosts)]./sum(edgeCosts);
denseTime = linspace(0,1,1000);

% determine time scaling to stay below velocity and acceleration limits
[~,~,~,~,~,~, vSpline, aSpline, ~, wSpline] ...
    = smoothPath(path, pathTime);

vScale = max(abs( vSpline(denseTime) )) / velocityLimit;
wScale = max(abs( wSpline(denseTime) )) / angularVelocityLimit;
aScale = sqrt(...
         max(abs( aSpline(denseTime) )) / accelerationLimit);
awScale = sqrt(...
         max(abs( diff(wSpline(denseTime))./diff(denseTime) )) ...
         / angularAccelerationLimit);

timeScale = max([ vScale wScale aScale awScale ]) / allowedScaleToLimit;

% scale time and calculate final smooth path
pathTime  = timeScale .* pathTime;
denseTime = timeScale .* denseTime;

[xSpline, ySpline, vxSpline, vySpline, axSpline, aySpline, ...
    vSpline, aSpline, angleSpline, wSpline] = smoothPath(path, ...
    pathTime);

% sSpline = @(t) integral(vSpline, 0, max(0, min(t, pathTime(end))));
% sTotal = sSpline(pathTime(end));

%% smooth path visualization

if false
figure(3)
subplot(2,1,1)
plot(denseTime, [axSpline(denseTime).'...
                 aySpline(denseTime).'], '.-')
legend('a_x','a_y', 'location','se')

subplot(2,1,2)
plot(denseTime, [vxSpline(denseTime).'...
                 vySpline(denseTime).'], '.-')
legend('v_x','v_y', 'location','se')


figure(4)
subplot(2,1,1)
plot(denseTime, [vSpline(denseTime); wSpline(denseTime)].', '.-')
legend('v','\omega', 'location','se')

subplot(2,1,2)
plot(denseTime, unwrap(angleSpline(denseTime))./pi, '.-')
legend('\theta/\pi', 'location','se')


figure(5)
show(map)
hold on
plot(path(:,1),path(:,2),'.-')
plot(xSpline(denseTime), ySpline(denseTime), '.-')
hold off

% deltat = denseTime(2) - denseTime(1);
% figure(10)
% plot(denseTime, unwrap([angleSpline(denseTime); ...
%                         deltat.*cumsum(wSpline(denseTime))].')./pi, '.-')
end

%% path following motion controller

close all

% initialization
XObs = [path(1,:) 0].';
PObs = Pinitial;
% XObs = [5 -2 pi/2].';
% XObs(3) = angleSpline(denseTime(2));
VObs = [0 0 0].';
VDes = VObs;
vDes = 0;
vw = [0 0].';
vwObs = [0 0].';
VError = 0;

if not(simulateOutput)
    
    ex3_initializeRosCommunication
    
    global odomTwist
    global cameraMsg
    global cameraMsg2
%     global odomPose
%     odomPose(:);
%     odomError = odomPose;
%     cameraImage = readImage(cameraMsg);
    actualPose = updateActualPose(gazeboSub);
    actualPose(:);
    actualX = actualPose;
else
    deltat = max(pathTime)/300;
end

if adaptiveTimeStep
    iterAmount = iterationLimit;
else
    iterAmount = floor(max(pathTime) / deltat);
end
landmarkOrderIndices = zeros(3,1);
seenLandmarks = [];
seenLandmarkAmount = 0;
savedX = NaN(3, iterAmount);
savedV = NaN(3, iterAmount);
savedT = NaN(iterAmount, 1);
savedOdomTwist = NaN(2, iterAmount);
savedActualX = NaN(3, iterAmount);
savedP3x3 = NaN(9, iterAmount);
savedM = NaN(6, iterAmount);
savedP = NaN(81, iterAmount);
savedScanResults = NaN(6, iterAmount);
waitTimeMin = Inf;
previousTime = 0;
t = 0;
tic;
for iteration = 1:iterAmount
    
    % desired poses and velocities
    XDes = [xSpline(t) ySpline(t) angleSpline(t)].';
    t = t + 0.5*deltat;
    VDesNext = [vxSpline(t) vySpline(t) wSpline(t)].';
    vDesNext = vSpline(t);
    t = t + 0.5*deltat;
    XDesNext = [xSpline(t) ySpline(t) angleSpline(t)].';
    t = t - deltat;
    
    % determine control velocities
    VErrorOld = VError;
    vwPrevious = vw;
    vw = controlVelocities(XObs(1:3), VObs, XDes, VDes, ...
                           XDesNext, VDesNext, vDesNext, ...
                           deltat, controlParams, ...
                           VErrorOld, vwPrevious, ...
                           velocityLimit, angularVelocityLimit, ...
                           accelerationLimit, angularAccelerationLimit);
    
    if simulateOutput
        % noisy observed output according to model
        vwObs = vw.*(0.8 + 0.3.*rand(2,1));
        XObs = f(XObs(1), XObs(2), XObs(3), ...
                 vwObs(1), vwObs(2), deltat);
        XObs = XObs + (0.03*rand(3,1) - 0.015)*deltat;
        VObs = [[cos(XObs(3)); sin(XObs(3))].*vwObs(1); vwObs(2)];
    else
        % send control velocities
        linear = [vw(1) 0 0].';
        angular = [0 0 vw(2)].';
        msg.Linear.X = linear(1);
        msg.Linear.Y = linear(2);
        msg.Linear.Z = linear(3);
        msg.Angular.X = angular(1);
        msg.Angular.Y = angular(2);
        msg.Angular.Z = angular(3);
        send(pub,msg)
        
        % synchronize time to match iteration time with deltat
        currentTime = toc;
        if adaptiveTimeStep
            deltat = currentTime - previousTime;
            previousTime = currentTime;
        else
            waitTime = deltat*iteration - currentTime;
            waitTimeMin = min(waitTimeMin, waitTime);
            if waitTime > 0
                pause(waitTime)
            end
        end
        
        vwObs = vw;
        actualXPrev = actualX; 
        actualX = updateActualPose(gazeboSub);
        if useOdomVelocitiesAsObservedVelocities
%             vwObs = odomTwist;
            vwObs(1) = odomTwist(1); % odom doesn't notice angular velocity
        elseif useActualVelocitiesAsObservedVelocities
            absAngleChange = abs(actualX(3) - actualXPrev(3));
            if absAngleChange < pi
                vwObs(2) = angdiff(actualXPrev(3), actualX(3)) ./ deltat;
                vwObs(1) = sqrt(sum( ...
                    ( (actualX(1:2) - actualXPrev(1:2))./deltat ).^2));
                vwObs(1) = vwObs(1) ./ archCorrectionFun(vwObs(2), deltat);
            end
        end
        
        % estimated state
        [XNext, PNext] = extendedKalmanPrediction(...
            XObs,PObs,vwObs,M,deltat,f,FFun,VFun);
        
        XObs = XNext;
        PObs = PNext;
        VObs = [[cos(XObs(3)); sin(XObs(3))].*vwObs(1); vwObs(2)];
        
        % measurement
%         [angles, ranges] = scanner.scanForLandmarks();
        cameraImage = readImage(cameraMsg);
        if useDepthImages
            depthImage = readImage(cameraMsg2);
            [angles, ranges] = ...
                scanForLandmarksOwn(cameraImage, depthImage);
        else
            [angles, ranges] = scanForLandmarksOwn(cameraImage);
        end
        measuredIndices = find(isfinite(angles));
                
        if not(isempty(measuredIndices))
            for lmIndex = squeeze(measuredIndices).'
                z = [ranges(lmIndex); angles(lmIndex)];
                if dataAssociationProblem
                    [XNext, PNext] = ...
                        extendedKalmanMeasurementDA(XObs,PObs,z,...
                                            landmarkPositions,R,h,HFun);
                elseif SLAM
                    % assign for a new landmark an order number index
                    if not(ismember(lmIndex, seenLandmarks))
                        landmarkOrderIndices(lmIndex) = ...
                            seenLandmarkAmount + 1;
                        isNewLandmark = true;
                    else
                        isNewLandmark = false;
                    end
                    
                    % accounting
                    seenLandmarks = union(seenLandmarks, lmIndex);
                    seenLandmarkAmount = numel(seenLandmarks);
                    landmarkOrderIndex = landmarkOrderIndices(lmIndex);
                    
                    % call Kalman function
                    [XNext, PNext] = extendedKalmanMeasurementSLAM(...
                        XObs,PObs,z,landmarkOrderIndex,R,h,HFun, ...
                        H2Fun,g,GFun,G2Fun,isNewLandmark);
                else
                    m = landmarkPositions(lmIndex,:);
                    [XNext, PNext] = ...
                        extendedKalmanMeasurement(XObs,PObs,z,m,R,h,HFun);
                end
                XObs = XNext;
                PObs = PNext;
            end
        end
    end
    
    % save progress
    savedX(:, iteration) = XObs(1:3);
    savedV(:, iteration) = VObs;
    savedT(iteration) = t;
    if not(simulateOutput)
%         XOdom = odomPose - odomError;
%         XOdom(3) = angdiff(odomError(3), odomPose(3));
%         XActual = updateActualPose(gazeboSub);
        XActual = actualX;
%         savedOdomX(:, iteration) = XOdom;
        savedOdomTwist(:, iteration) = odomTwist;
        savedActualX(:, iteration) = XActual;
        P3x3 = PObs(1:3,1:3);
        savedP3x3(:, iteration) = P3x3(:);
        
        savedM(1:numel(XObs)-3, iteration) = XObs(4:end);
        Pnanpadded = NaN(sqrt(size(savedP,1)));
        Pnanpadded(1:numel(XObs), 1:numel(XObs)) = PObs;
        savedP(:, iteration) = Pnanpadded(:);
        
        savedScanResults(:, iteration) = ...
            [g(XActual(1), XActual(2), XActual(3), ranges(1), angles(1))
             g(XActual(1), XActual(2), XActual(3), ranges(2), angles(2))
             g(XActual(1), XActual(2), XActual(3), ranges(3), angles(3))];
    end
    
    % update for next loop
    VDes = VDesNext;
    vDes = vDesNext;
    t = t + deltat; % or current time
    if t + deltat >= pathTime(end)
        break
    end
end

% final message
if not(simulateOutput)
    linear  = [0 0 0].';
    angular = [0 0 0].';
    msg.Linear.X = linear(1);
    msg.Linear.Y = linear(2);
    msg.Linear.Z = linear(3);
    msg.Angular.X = angular(1);
    msg.Angular.Y = angular(2);
    msg.Angular.Z = angular(3);
    send(pub,msg)
end

%% visualization of the simulation

figure(6)
show(map)
hold on
plot(path(:,1),path(:,2),'.-')
plot(xSpline(denseTime), ySpline(denseTime), '.-')
plot(savedX(1,:), savedX(2,:), '.-')
if not(simulateOutput)
    plot(savedActualX(1,:), savedActualX(2,:), '.-')
    plot(savedScanResults([3 5 1],1:iteration).', ...
         savedScanResults([4 6 2],1:iteration).', 'o-')
end
legend('unsmooth path', 'smooth path', ...
    'observed path', 'actual path', ...
    'actual observation 1', ...
    'actual observation 2', ...
    'actual observation 3', 'location', 'se')
hold off

figure(7)
plot(savedT, [xSpline(savedT) ySpline(savedT) ...
              unwrap(angleSpline(savedT)) ...
              savedX([1 2 3],:).'], '.-');
xlabel('time (s)')
ylabel('position')
legend('path x', 'path y', 'path \theta', ...
    'observed x', 'observed y', 'observed \theta', ...
    'location', 'se')
figure(8)
plot(savedT, [vxSpline(savedT) vySpline(savedT) wSpline(savedT) ...
              savedV([1 2 3],:).'], '.-');
xlabel('time (s)')
ylabel('velocity')
legend('path v_x', 'path v_y', 'path \omega', ...
    'observed v_x', 'observed v_y', 'observed \omega', ...
    'location', 'sw')

if and(not(simulateOutput), SLAM)
    figure(16)
    show(map)
    hold on
    plot(path(:,1),path(:,2),'.-')
    plot(xSpline(denseTime), ySpline(denseTime), '.-')
    plot(savedX(1,:), savedX(2,:), '.-')
    plot(savedActualX(1,:), savedActualX(2,:), '.-')
    plot(savedM([3 1 5],:).', savedM([4 2 6],:).', '.-')
    legend('unsmooth path', 'smooth path', ...
        'observed path', 'actual path', 'landmark estimate 1', ...
        'landmark estimate 2', 'landmark estimate 3', 'location', 'se')
    hold off
    
    figure(18)
    plot(savedT, savedM([5 6 3 4 1 2],:).', '.-');
    hold on
    plot(0,0,'-')
    plot(savedT, savedScanResults(:, 1:iteration).', 'o');
    plot(0,0,'-')
    plot(savedT([1 end]), [landmarkPositions([1 4 2 5 3 6]); ...
                           landmarkPositions([1 4 2 5 3 6])], ':');
    plot(0,0,'-')
    plot(savedT, savedM([5 6 3 4 1 2],:).' ...
                 + 1.645.*sqrt(savedP([71 81 51 61 31 41], ...
                                      1:iteration)).', '-.');
    plot(0,0,'-')
    plot(savedT, savedM([5 6 3 4 1 2],:).' ...
                 - 1.645.*sqrt(savedP([71 81 51 61 31 41], ...
                                      1:iteration)).', '-.');
    hold off
    xlabel('time (s)')
    ylabel('landmark position coordinate')
    title(': true, o actual measurement, .- estimate, -. 90% confidence')
    legend('red x', 'red y', 'green x', 'green y', ...
           'blue x', 'blue y', 'location', 'se')
end

if not(simulateOutput)
    figure(19)
    savedP3x3Eigen = zeros(3, length(savedT));
    for tIndex = 1:length(savedT)
        savedP3x3Eigen(:,tIndex) = eig(reshape(savedP3x3(:,tIndex),[3 3]));
    end
    plot(savedT, prod(savedP3x3Eigen.', 2), '.-');
    xlabel('time (s)')
    ylabel('product of robot pose covariance eigenvalues')
    set(gca,'yscale','log')
    
%     figure(9)
%     plot(savedT, savedP.', '.-');
%     xlabel('time (s)')
%     ylabel('variance')
%     legend('var(x)','var(y)','var(\theta)')
    
%     figure(10)
%     plot(savedT, [unwrap(angleSpline(savedT)) savedX(3,:).' ...
%         unwrap(savedActualX(3,:)).' savedOdomX(3,:).'], '.-');
%     xlabel('time (s)')
%     ylabel('angle (rad)')
%     legend('planned \theta', 'observed \theta', ...
%            'actual \theta', 'odometry \theta')
end

% set(gcf, 'PaperUnits', 'points', ...
% 'PaperPosition', [0 0 432 324], 'PaperSize', [432 324])
% print('fomr_ex2_1', '-dmeta')


%% functions
function [xNext, PNext] = extendedKalmanPrediction(x,P,vw,M,dt,f,FFun,VFun)
    
    % prediction
    F = eye(size(P));
    F(1:3,1:3) = FFun(x(1),x(2),x(3),vw(1),vw(2),dt);
    V = VFun(x(1),x(2),x(3),vw(1),vw(2),dt);
    Q = zeros(size(P));
    Q(1:3,1:3) = V*M*V';
    xNext = x;
    xNext(1:3) = f(x(1),x(2),x(3),vw(1),vw(2),dt);
    PNext = F*P*F' + Q;
end
function [xNext, PNext] = extendedKalmanMeasurement(x,P,z,m,R,h,HFun)
    
    % include measurement to estimation
    I = eye(size(P));
    H = HFun(x(1),x(2),x(3),m(1),m(2));
    hx = h(x(1),x(2),x(3),m(1),m(2));
    y = z - hx;
    y(2) = angdiff(hx(2), z(2));
    S = R + H*P*H';
    K = P*H'/S;
    xNext = x + K*y;
    PNext = (I - K*H)*P;
end
function [xNext, PNext] = extendedKalmanMeasurementSLAM(...
    x,P,z,landmarkIndex,R,h,HFun,H2Fun,g,GFun,G2Fun, isNewLandmark)
    % simultaneous localization and mapping
    
    % test if measurement refers to a new landmark
    mIndicesInX = (4:5) + 2*(landmarkIndex-1);
    if isNewLandmark
        m = g(x(1),x(2),x(3),z(1),z(2));
        
        H  =  HFun(x(1),x(2),x(3),m(1),m(2));
        G  =  GFun(x(1),x(2),x(3),z(1),z(2));
        G2 = G2Fun(x(1),x(2),x(3),z(1),z(2));
        P3x3 = P(1:3,1:3);
        Pnew2x2part = G*P3x3*G' + G2*(H*P3x3*H' + R)*G2';
        P = padarray(P,2,0,'post');
        P(mIndicesInX, mIndicesInX) = Pnew2x2part;
        
        xNext = [x; m];
        PNext = P;
        return
    end
    
    % define m and some matrices before Kalman algorithm
    m = x(mIndicesInX);
    I = eye(size(P));
    H = zeros(2, length(x));
    H(:,1:3) = HFun(x(1),x(2),x(3),m(1),m(2));
    H(:,mIndicesInX) = H2Fun(x(1),x(2),x(3),m(1),m(2));
    
    % Kalman, include measurement to estimation
    hx = h(x(1),x(2),x(3),m(1),m(2));
    y = z - hx;
    y(2) = angdiff(hx(2), z(2));
    S = R + H*P*H';
    K = P*H'/S;
    xNext = x + K*y;
    PNext = (I - K*H)*P;
%     PNext = (I - K*H)*P*(I - K*H)' + K*R*K';
end
function [xNext, PNext] = extendedKalmanMeasurementDA(...
                          x,P,z,landmarkPositions,R,h,HFun)
    % data association
    
    % include measurement to estimation
    I = eye(size(P));
    ySy = Inf;
%     landmarkPositions = [5 5; 1 2.5; -5.5 0];
    lmAmount = size(landmarkPositions, 1);
    for lmIndex = 1:lmAmount
        m = landmarkPositions(lmIndex, :);
        H_ = HFun(x(1),x(2),x(3),m(1),m(2));
        hx = h(x(1),x(2),x(3),m(1),m(2));
        y_ = z - hx;
        y_(2) = angdiff(hx(2), z(2));
        S_ = R + H_*P*H_';
        ySy_ = (y_'/S_)*y_;
        if ySy_ < ySy
            ySy = ySy_;
            y = y_;
            S = S_;
            H = H_;
        end
    end
    K = P*H'/S;
    xNext = x + K*y;
    PNext = (I - K*H)*P;
end

function [xSpline, ySpline, vxSpline, vySpline, axSpline, aySpline, ...
    vSpline, aSpline, angleSpline, wSpline] = smoothPath(path, ...
    pathTime)

    xSplineFunction = spline(pathTime, [0; path(:,1); 0]);
    ySplineFunction = spline(pathTime, [0; path(:,2); 0]);

    xSpline = @(t) ( ppval(xSplineFunction, t) );
    ySpline = @(t) ( ppval(ySplineFunction, t) );

    vxSpline = @(t) ( ppval(fnder(xSplineFunction,1), t) );
    vySpline = @(t) ( ppval(fnder(ySplineFunction,1), t) );

    axSpline = @(t) ( ppval(fnder(xSplineFunction,2), t) );
    aySpline = @(t) ( ppval(fnder(ySplineFunction,2), t) );

    vSpline = @(t) (sqrt(vxSpline(t).^2 + vySpline(t).^2));
    aSpline = @(t) (sqrt(axSpline(t).^2 + aySpline(t).^2));
    angleSpline = @(t) (angle(vxSpline(t) + 1i.*vySpline(t)));
    wSpline = @(t) ((vSpline(t) > 1e-12) .* ...
                    (aySpline(t).*vxSpline(t) - ...
                     axSpline(t).*vySpline(t)) ./ ...
                    (vSpline(t).^2 + (vSpline(t) <= 1e-12)));
end

function vw = controlVelocities(XObs, VObs, XDes, VDes, ...
                                XDesNext, VDesNext, vDesNext, ...
                                deltat, controlParams, ...
                                VErrorOld, vwPrevious, ...
                                velocityLimit, angularVelocityLimit, ...
                                accelerationLimit, ...
                                angularAccelerationLimit)
    % error in angular velocity when target direction
    % is the angle of -XErrorNext([1;2]) vector
    XErrorNext = (XObs - XDesNext) ./ deltat; % theta-component not used
    errorAngle = ...
        angdiff( atan2(-XErrorNext(2), -XErrorNext(1)), XObs(3) );
    WError = [0; 0; errorAngle/deltat + VDesNext(3)];
    
    % errors from pose and velocities
    XError = (XObs - XDes) ./ deltat;
    XError(3) = angdiff(XDes(3), XObs(3)) / deltat;
%     VErrorOld = VError;
    VError = VObs - VDes;
    
    % condition coefficients
    misalignmentAngle = ...
        angdiff( atan2(-XErrorNext(2), -XErrorNext(1)), XDes(3) );
    vErr = sqrt(XError(1)^2 + XError(2)^2);
    alignedErrorAndPath = 0.5 + 0.5*cos(misalignmentAngle);
    parallelToError = 0.5 + 0.5*cos(2*errorAngle);
    farFromPath = vErr / ( vErr + vDesNext + (0==vErr) );
    
    % hold down correction in certain conditions
    WError = (1-(1-alignedErrorAndPath)*(1-farFromPath)) .* WError;
    XError(1:2) = (1-(1-parallelToError)*farFromPath) .* XError(1:2);
    XError(3) = (1-farFromPath) * XError(3);
    VError = alignedErrorAndPath.*(1-farFromPath) .* VError;
    VError = VError + VErrorOld; % integration
    
    % control velocity as [vx; vy; w]
    VControl = VDesNext - controlParams(1).*XError ...
                        - controlParams(2).*VError ...
                        - controlParams(3).*WError;
    
    % control velocities bounded by velocity limits
    limitScaleV = min(1, ...
        velocityLimit ./ sqrt( VControl(1)^2 + VControl(2)^2 ));
    limitScaleW = min(1, angularVelocityLimit ./ abs( VControl(3) ));
    limitScale = [limitScaleV; limitScaleV; limitScaleW];
    VControl = limitScale .* VControl;
    
    % control speed and angular velocity
%     vwPrevious = vw;
    vw = [sqrt( VControl(1)^2 + VControl(2)^2 )
          VControl(3)];
    
    % limit change in velocities
    vwDiff = vw - vwPrevious;
    vwDiffLimits = [accelerationLimit; angularAccelerationLimit].*deltat;
    aboveLimit = abs(vwDiff) > vwDiffLimits;
    vw(aboveLimit) = vwPrevious(aboveLimit) ...
        + sign(vwDiff(aboveLimit)).*vwDiffLimits(aboveLimit);
    
    % limit total acceleration
    vwDiff = vw - vwPrevious;
    accelTotalSq = (vwDiff(1)/deltat)^2 + (vw(1)*vw(2))^2;
    if accelTotalSq > accelerationLimit^2
        vw = vwPrevious ...
            + vwDiff .* (accelerationLimit/sqrt(accelTotalSq));
    end
end
