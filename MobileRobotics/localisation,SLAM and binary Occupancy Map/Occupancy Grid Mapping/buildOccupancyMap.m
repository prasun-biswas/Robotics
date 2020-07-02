%% function to build the binary occupancy map
% get binary map as input and output updated binary map
function updated_binary_map = buildOccupancyMap(binary_map)
    % Subscribe to the global variables for position and laserscan
    global odomPose
    global laser_msg
    % extract the x,y,theta of turtlebot from odometry data
    turtlebot_x = odomPose(1);
    turtlebot_y = odomPose(2);
    turtlebot_theta = odomPose(3);
    % display the location of turtlebot in console
    disp(turtlebot_x)
    disp(turtlebot_y)
    % get laserscan paramters
    laserscan_angle_min = laser_msg.AngleMin;
    laserscan_angle_max = laser_msg.AngleMax;
    laserscan_range_min = laser_msg.RangeMin;
    laserscan_range_max = laser_msg.RangeMax;
    % get laserscan measurement
    laserscan_measurement = laser_msg.Ranges;
    % get range of laser scan (=~60  degrees)
    laser_angle_range = laserscan_angle_max - laserscan_angle_min; 
    % half of laser scan range
    half_laser_angle_range = laser_angle_range/2;
    % get the number of segments available for laser scan
    total_laser_segment = size(laserscan_measurement,1);
    % get the segment angle between each segment
    segment_angle = round(laser_angle_range/total_laser_segment,4);
    % create zero matrix of size [number of laser segment, 2]
    laserscan_with_angle = zeros(size(laserscan_measurement,1),2);
    % fill the column 2 of the above matrix with laserscan measurement from
    % each segment
    laserscan_with_angle(:,2) = laserscan_measurement;
    % fill col 1 with the segment angle for each laserscan measurement 
    for iter = 1:size(laserscan_measurement,1)
        laserscan_with_angle(iter,1) = (iter-1)*segment_angle + turtlebot_theta - half_laser_angle_range;
    end
    % create and [1,2] matrix having the present x,y of turtlebot
    turtlebot_xy = [turtlebot_x turtlebot_y];
    % get the location of turtlebot in the grid of size [781,1330]
    ij = world2grid(binary_map, turtlebot_xy);
    % define an observation zone based on the orientation of turtlebot and
    % define range of cols and rows to be considered for building the map.
    % This is done to reduce the computation time as w.r.t turtlebot
    % position we are neglecting the zones where no observation is
    % possible.
    % if 0 <= orientation <= 90 then the turtlebot is in 1st quadrant and
    % the observation zone is towards right and up direction
    if (0 <= turtlebot_theta) && (turtlebot_theta <= 1.5708)
        disp('in 1') % display in 1st quadrant
        % create a variable that is 4 steps towards right from current
        % position or it is the maximum limit of X-axis for the map
        if(turtlebot_x + 4) <= binary_map.XWorldLimits(1,2)
            turtlebot_x_temp = turtlebot_x + 4;
        else
            turtlebot_x_temp = binary_map.XWorldLimits(1,2);
        end
        % create a variable that is 4 steps towards up from current
        % position it is the maximum limit of Y-axis for the map
        if(turtlebot_y + 4) <= binary_map.YWorldLimits(1,2)
            turtlebot_y_temp = (turtlebot_y + 4);
        else
            turtlebot_y_temp = binary_map.YWorldLimits(1,2);
        end
        % create a temporary matrix with observation coordinates for
        % turtlebot
        turtlebot_xy_temp = [turtlebot_x_temp turtlebot_y_temp];
        % convert from cartesian coordinates to grid coordinates
        ij_temp = world2grid(binary_map, turtlebot_xy_temp);
        % set the range of rows and columns to consider
        selected_binary_map_rows_min = ij_temp(1,1);
        selected_binary_map_rows_max = ij(1,1);
        selected_binary_map_cols_min = ij(1,2);
        selected_binary_map_cols_max = ij_temp(1,2);
        
    % if 90 < orientation <= 180 then the turtlebot is in 2nd quadrant and
    % the observation zone is towards left and up direction
    elseif (1.5708 < turtlebot_theta) && (turtlebot_theta <= 3.14159)
        disp('in 2') % display in 2nd quadrant
        % create a variable that is 4 steps towards left from current
        % position or it is the minimum limit of X-axis for the map
        if(turtlebot_x - 4) >= binary_map.XWorldLimits(1,1)
            turtlebot_x_temp = turtlebot_x - 4;
        else
            turtlebot_x_temp = binary_map.XWorldLimits(1,1);
        end
        % create a variable that is 4 steps towards up from current
        % position it is the maximum limit of Y-axis for the map
        if(turtlebot_y + 4) <= binary_map.YWorldLimits(1,2)
            turtlebot_y_temp = (turtlebot_y + 4);
        else
            turtlebot_y_temp = binary_map.YWorldLimits(1,2);
        end
        % create a temporary matrix with observation coordinates for
        % turtlebot
        turtlebot_xy_temp = [turtlebot_x_temp turtlebot_y_temp];
        % convert from cartesian coordinates to grid coordinates
        ij_temp = world2grid(binary_map, turtlebot_xy_temp);
        % set the range of rows and columns to consider
        selected_binary_map_rows_min = ij_temp(1,1);
        selected_binary_map_rows_max = ij(1,1);
        selected_binary_map_cols_min = ij_temp(1,2);
        selected_binary_map_cols_max = ij(1,2);
    % if -90 < orientation < 0 then the turtlebot is in 3rd quadrant and
    % the observation zone is towards right and down direction
    elseif (-1.5708 <= turtlebot_theta) && (turtlebot_theta < 0)
        disp('in 3') % display in 3rd quadrant
        % create a variable that is 4 steps towards right from current
        % position or it is the maximum limit of X-axis for the map
        if(turtlebot_x + 4) <= binary_map.XWorldLimits(1,2)
            turtlebot_x_temp = turtlebot_x + 4;
        else
            turtlebot_x_temp = binary_map.XWorldLimits(1,2);
        end
        % create a variable that is 4 steps towards down from current
        % position it is the minimum limit of Y-axis for the map
        if(turtlebot_y - 4) >= binary_map.YWorldLimits(1,1)
            turtlebot_y_temp = (turtlebot_y - 4);
        else
            turtlebot_y_temp = binary_map.YWorldLimits(1,1);
        end
        % create a temporary matrix with observation coordinates for
        % turtlebot
        turtlebot_xy_temp = [turtlebot_x_temp turtlebot_y_temp];
        % convert from cartesian coordinates to grid coordinates
        ij_temp = world2grid(binary_map, turtlebot_xy_temp);
        % set the range of rows and columns to consider
        selected_binary_map_rows_min = ij(1,1);
        selected_binary_map_rows_max = ij_temp(1,1);
        selected_binary_map_cols_min = ij(1,2);
        selected_binary_map_cols_max = ij_temp(1,2);
    % if -180 <= orientation < -90 then the turtlebot is in 4th quadrant and
    % the observation zone is towards left and down direction
    elseif (-3.14159 <= turtlebot_theta) && (turtlebot_theta < -1.5708)
        disp('in 4') % display in 4th quadrant
        % create a variable that is 4 steps towards left from current
        % position or it is the minimum limit of X-axis for the map
        if(turtlebot_x - 4) >= binary_map.XWorldLimits(1,1)
            turtlebot_x_temp = turtlebot_x - 4;
        else
            turtlebot_x_temp = binary_map.XWorldLimits(1,1);
        end
        % create a variable that is 4 steps towards down from current
        % position it is the minimum limit of Y-axis for the map
        if(turtlebot_y - 4) >= binary_map.YWorldLimits(1,1)
            turtlebot_y_temp = turtlebot_y - 4;
        else
            turtlebot_y_temp = binary_map.YWorldLimits(1,1);
        end
        % create a temporary matrix with observation coordinates for
        % turtlebot
        turtlebot_xy_temp = [turtlebot_x_temp turtlebot_y_temp];
        % convert from cartesian coordinates to grid coordinates
        ij_temp = world2grid(binary_map, turtlebot_xy_temp);
        % set the range of rows and columns to consider
        selected_binary_map_rows_min = ij(1,1);
        selected_binary_map_rows_max = ij_temp(1,1);
        selected_binary_map_cols_min = ij_temp(1,2);
        selected_binary_map_cols_max = ij(1,2);
    end
    disp('building') % display map building
    % iterate over the binary map defined by the range of rows and columns
    % as calculated above
    for iter1 = selected_binary_map_rows_min:selected_binary_map_rows_max
        for iter2 = selected_binary_map_cols_min:selected_binary_map_cols_max
            % create a temporary matrix of dimesion [1,2] with the
            % iterator having the values of selected rows and cols. 
            ij = [iter1 iter2];
            % convert the grid coordinates for the selected cell to
            % cartesian coordinates
            xy = grid2world(binary_map, ij);
            % get the cell angle w.r.t the turtlebot in world-frame
            cell_angle = atan2((xy(2)-turtlebot_y), (xy(1)-turtlebot_x));
            % get distance from the turtlebot
            cell_distance = sqrt((xy(2) - turtlebot_y).^2 +...
                                (xy(1) - turtlebot_x).^2);
            % check if the cell is located inside the laserscan
            % measurement range
            if (-half_laser_angle_range + turtlebot_theta <= cell_angle) && ...
               (cell_angle <= half_laser_angle_range + turtlebot_theta)...
                 && (laserscan_range_min <= cell_distance) && (cell_distance <= laserscan_range_max)
                % if cell is in laserscan range then find the nearest
                % laserscan segment observation.
                [~, index_match] = min(abs(laserscan_with_angle(:,1) - cell_angle));
                % get the laserscan measurement data corresponding to the
                % cell
                laserscan_measurement = laserscan_with_angle(index_match, 2);
                % call inverse_sensor_model to get the occupanc of the cell
                occupancy_state = inverse_sensor_model(xy, turtlebot_xy, laserscan_measurement); 
                % if occupancy is 1 then set it to 1
                if occupancy_state == 1
                    setOccupancy(binary_map, xy, 1)
                end
            end
        end
    end
    % update the binary map and return it to the calling function.
    updated_binary_map = binary_map;
end