% reconnect with ros
% rosshutdown
% rosinit
ex3_initializeRosCommunication

% load "moroparams.mat" and show the map
figure
load('moroparams.mat')
hold 
show(map)

% initialise scanner object and define landmark
scanner = MoroLandmarkScanner();
landmark = [[5,5];[1,2.5];[-5.5,0]];
%% initialising the parameters of particle filter
world_size = map.GridSize(2)/(map.XWorldLimits(2)-map.XWorldLimits(1)); % workspace area
N = world_size*5; % number of particles
T = world_size/5; % number of iterations
myRobot = Robot(); % creating robot object
empty_robot = Robot(); % creating robot object
p(1,N) = empty_robot; % creating an row-matrix of N robot object
p2(1,N) = empty_robot; % creating an row-matrix of N robot object
p3(1,N) = empty_robot; % creating an row-matrix of N robot object
w = zeros(1,N); % particle weight
dt = 0.01; % refresh time  
%% infinite loop to perfrom localisation
% counter = 0;
comparisionData = [];
actualX = [0 0 0].';
currentTime = 0;
tic;
while 1 
    % creating N random numbers in the workspace range
    x_world_range = (map.XWorldLimits(1,2) - map.XWorldLimits(1,1))*rand(N,1) + map.XWorldLimits(1,1);
    y_world_range = (map.YWorldLimits(1,2) - map.YWorldLimits(1,1))*rand(N,1) + map.YWorldLimits(1,1);
    
    % setting parameters for the N robots (particles) created above
    for i = 1:N
        r = Robot();
        r.set(x_world_range(i), y_world_range(i), rand * 2.0 * pi);
        r.set_noise(0.05, 0.05, 1)
        p(i) = r;
    end  
    [A,R] = scanner.scanForLandmarks(); % angle,range reading from landmark scanner object
    [x_cord,y_cord] = pol2cart(A,R);  % converting to cartesian coordinates
    x_pt_index = find(~isnan(x_cord)); % get the index of landmark
    y_pt_index = find(~isnan(y_cord)); % get the index of landmark
    
    % get time between the current observation and present observation
    previousTime = currentTime;
    currentTime = toc;
    % get the time difference
    deltat = currentTime - previousTime;
    % compare current location and previous location
    actualXPrev = actualX; 
    actualX = updateActualPose(gazeboSub);
    absAngleChange = abs(actualX(3) - actualXPrev(3));
    % get angular and linear velocity
    w_angular = angdiff(actualXPrev(3), actualX(3)) ./ deltat;
    v_linear = sqrt(sum(((actualX(1:2) - actualXPrev(1:2))./deltat ).^2));
    theta = actualX(3);
    
    if(~isnan(x_pt_index)) % if landmark visible do localisation
        % iterating over the N particles to select the best particles which follows myRobot. 
        for t = 1:T
            % get location from landmark reading
            X_turtlebot = landmark(x_pt_index,1) - R(x_pt_index)*cos(theta + A(x_pt_index));
            Y_turtlebot = landmark(y_pt_index,2) - R(y_pt_index)*sin(theta + A(y_pt_index));
            myRobot.set(X_turtlebot, Y_turtlebot, theta) % set robot position
            myRobot = myRobot.move(w_angular, v_linear, dt); % get motion data
            Z = myRobot.sense(landmark); % measure location of robot w.r.t landmarks
            % move the N particle in accordance with myRobot (Robot object)
            for i = 1:N
                p2(i) = p(i).move(w_angular, v_linear, dt);
            end
            % calculate weight of the particles (particles closer to myRobot have higher weight)
            p = p2;
            for i = 1:N
                w(i) = p(i).measurement_prob(Z, landmark);
            end
            % resampling of particles using resampling wheel algorithm
            index = randi(N); % select random index
            beta = 0.0; % initialise beta 
            max_w = max(w); % find the particle maximum weight
            % iterate over all the particles
            for i = 1:N
                beta = beta + rand*2*max_w; 
                while beta > w(index)
                    beta = beta - w(index);
                    index = mod((index + 1) ,N) + 1;          
                end
                p3(i) = p(index);
            end
            p = p3;
            % create matrix for plotting the selected particles
            plot_point = zeros(N,2);
            for i = 1:N
                plot_point(i,1) = p(1,i).x;
                plot_point(i,2) = p(1,i).y;
            end
        end
        % plot x, y from odom data
        x = actualX(1);
        y = actualX(2);
        plot_1 = plot(x, y, 'r--*', 'MarkerSize', 10);
        % plot particles
        plot_2 = plot(plot_point(:,1),plot_point(:,2),'.');
        x_mean = mean(plot_point(:,1));
        y_mean = mean(plot_point(:,2));
        comparisionData = [comparisionData; x, y, x_mean, y_mean];
        pause(1)
        delete(plot_1) % delete the current plot
        delete(plot_2) % delete the current plot
    else
        disp('no landmark visible')
    end
    pause(0.01)
end

    

