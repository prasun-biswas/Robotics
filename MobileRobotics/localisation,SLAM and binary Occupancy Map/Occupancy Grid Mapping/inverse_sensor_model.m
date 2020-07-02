%% inverse sensor model to decide the occupancy of cells
% get the cell, turtlebot pose and laserscan measuremenent from calling
% function and return occupancy value
function occ = inverse_sensor_model(grid_cell, turtlebot_pose, laser_measurement)
    % get the distance of cell from the current turtlebot location
    r = sqrt((grid_cell(2) - turtlebot_pose(2)).^2 + (grid_cell(1) - turtlebot_pose(1)).^2);  
    % as the laser scan can see only the surface of the objects we are
    % taking an offset to define the thickness of object
    offset_thickness = 0.1;
    % check if the distance of cell is in range of the laserscan
    % measurement with some offset
    if (r >= laser_measurement - offset_thickness) && (r <= laser_measurement + offset_thickness)
        % set occupancy to 1
        occ = 1;
    else
        % set occupancy to 0
        occ = 0;
    end
end
