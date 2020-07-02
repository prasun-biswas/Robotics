%% initialise binary occupancy grid map
moro_map = load('moroparams.mat'); % load the provided .mat file
binary_map = moro_map.map; % extract the map
figure % create new figure
show(binary_map) % show the map
% get rows and cols used in the map
binary_map_rows = binary_map.GridSize(1);
binary_map_cols = binary_map.GridSize(2);
% create an zero matrix of dimension [(number of rows)*(number of cols),2]
vector_binary_map = zeros(binary_map_rows*binary_map_cols,2);
% fill the values of rows and cols into the zero matrix created above
counter = 1;
for iter1 = 1:binary_map_rows
    for iter2 = 1:binary_map_cols
        vector_binary_map(counter,1) = iter1;
        vector_binary_map(counter,2) = iter2;
        counter = counter + 1;
    end
end

%% initialise ROS communication
% for this program we are using 'odomCallback' and 'laserscanCallback'for
% receiving the position and laserscan measurement of the turtlebot
ex3_initializeRosCommunication
%% initialise map building with user input
% create another map of the same dimension as the map provided but with  
% binary occupancy set to 0 for all cells
setOccupancy(binary_map, vector_binary_map, 0, 'grid')
figure;
show(binary_map);
% initiate a infinite while loop asking for user input to proceed with map
% building.(press 1 to build map, press 0 to exit, other input will again 
% ask for proper user input)
while 1
    prompt = 'press 1 to proceed and 0 to exit';
    user_input = input(prompt);
    if user_input == 0
        break
    elseif user_input == 1 
        % if user input is 1 call function buildOccupancyMap with the
        % current binary map.
        binary_map = buildOccupancyMap(binary_map);
        show(binary_map)
        pause(1)
    else
        continue
    end
end
