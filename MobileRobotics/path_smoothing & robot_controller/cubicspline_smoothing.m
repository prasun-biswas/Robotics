function [location_traj, velocity_traj, accelaration_traj] = cubicspline_smoothing(segments,init_loc, init_vel, init_acc, final_loc, final_vel, final_acc, P, Pdot, Pddot, path_desired, wt )
    n_Pcol=size(P,2); % get number of columns of P
    temp1=P(1,:);       % temporary array with the first row of P matrix
    temp2=P(end,:);   % temporary array with the last row of the P matrix
    temp3=Pdot(1,:);    % temporary array with the first row of the Pdot matrix
    temp4=Pdot(end,:); % temporary array with the last row of the Pdot matrix
    temp5=Pddot(1,:);   % temporary array with the first row of the Pddot matrix
    temp6=Pddot(end,:); % temporary array with the last row of the Pddot matrix

% stitching constraints in the form AX=B where X is the variable
% initialise an empty matrix to compute optimisation
A_eq_path=[];

for i=1:segments-1
    A_pos_path=[temp2 -temp1];    % position level stitching
    A_vel_path=[temp4 -temp3];    % velocity level stitching
    A_acc_path=[temp6 -temp5];    % acceleration level stitching
    % vetically stacking the stitching contraints for the present segment 
    % inserting zeros at the front or back A_eq_path_temp
    % A_eq_path_temp has 1442 (14(=number of segments)X101(=number of steps))
    % columns consiting of data part in two columns the rest columns are zeros
    A_eq_path_temp=[A_pos_path;A_vel_path;A_acc_path];
    A_eq_path_temp=[zeros(size(A_eq_path_temp,1),(i-1)*n_Pcol) A_eq_path_temp zeros(size(A_eq_path_temp,1),(segments-i-1)*n_Pcol)]; % filling the irrelevant matrix part with zeroes
    % appending all stitching constraints in one final matrix resulting in
    % a matrix of dimension (39(=number of segments-1*3) X 1442 (=number of columns of A_eq_path_temp))
    A_eq_path = [A_eq_path; A_eq_path_temp]; 
end


%INITIAL BOUNDARY CONSTRAINTS from temp variable created above from P
% matrix resulting in matrix dimension 1 X 1442 (14(=number of segments)X101(=number of steps))
% being the starting point first column is filled with temp values and the
% remaining are zeros
A_init_pos_boundary = [temp1  zeros(1,(segments-1)*n_Pcol)];   % Initial location boundary constraints.  
A_init_vel_boundary = [temp3  zeros(1,(segments-1)*n_Pcol)];   % Initial velocity constraints. 
A_init_acc_boundary = [temp5  zeros(1,(segments-1)*n_Pcol)];   % Inital acceleration boundary constraints.  

% FINAL BOUNDARY CONSTRAINTS from temp variable created above from P
% matrix in matrix dimension 1 X 1442 (14(=number of segments)X101(=number of steps))
% being the end point last column is filled with temp values and the
% remaining are zeros
A_final_pos_boundary = [zeros(1,(segments-1)*n_Pcol)  temp2];   % Final position constraints. 
A_final_vel_boundary = [zeros(1,(segments-1)*n_Pcol)  temp4];   % Final velocity constraints. 
A_final_acc_boundary = [zeros(1,(segments-1)*n_Pcol)  temp6];   % Final acceleration constraints. 

% vertically stacking the boundary conditions
A_eq_boundary = [A_init_pos_boundary; A_init_vel_boundary; A_init_acc_boundary; A_final_pos_boundary; A_final_vel_boundary; A_final_acc_boundary ]; % statcking all boundary constraints in one matrix.


%constructing the B matrix of the stitching and boundary constraints
B_eq_path=zeros((segments-1)*3,1);

B_init_pos_boundary = init_loc;     % initial position boundary constraints
B_init_vel_boundary = init_vel;  % initial velocity boundary constraints
B_init_acc_boundary = init_acc; % initial acceleration constraints
B_final_pos_boundary = final_loc;     % final position constraints
B_final_vel_boundary = final_vel;  % final velocity constraints
B_final_acc_boundary = final_acc; % final acceleration constraint
% vertically stacking the boundary conditions
B_eq_boundary = [ B_init_pos_boundary; B_init_vel_boundary; B_init_acc_boundary; B_final_pos_boundary; B_final_vel_boundary; B_final_acc_boundary ]; % stacking B matrix of boundary constraints in one big matrix

% stacking stitching and boundary constraints together
A_eq = [ A_eq_path; A_eq_boundary]; 
B_eq = [ B_eq_path; B_eq_boundary ];

%construction of objective function
cost_smoothness=eye(n_Pcol*(segments));   % Q_{smoothness} 
lincost_smoothness = zeros( n_Pcol*(segments), 1) ;  % q_{smoothness} 
tracking_term=P;
blkdiagPdot=Pdot;
blkdiagPddot=Pddot;
for i=2:segments    
    tracking_term=blkdiag(tracking_term,P);
    blkdiagPdot=blkdiag(blkdiagPdot,Pdot);
    blkdiagPddot=blkdiag(blkdiagPddot,Pddot);
end

cost_tracking = tracking_term'* tracking_term;      % Q_{tracking}
lincost_tracking = -tracking_term'*path_desired;    % q_{tracking}
cost = wt*cost_smoothness+cost_tracking ;       % w*Q_{smoothness}+Q_{tracking}
lincost = wt*lincost_smoothness+lincost_tracking;   % w*q_{smoothness}+q_{tracking}

x = quadprog(cost,lincost,[],[],A_eq,B_eq); % solving the optimization problem
location_traj = tracking_term*x;  % final position trajectory
velocity_traj=blkdiagPdot*x;   % final velocity trajectory
accelaration_traj=blkdiagPddot*x; % final acceleration trajectory

end

