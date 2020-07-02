%needs files: same_location_successors.m and edge_cost.m and
%orientation_min2max.m

%start_node= [5, 29, 0] end_node= [29, 20, 0]

function [min_cost, route, route_found] = dijkstra(n0, ng, N, A) 

%% initialize node n0 and ng
    %n0 
    %if same orientations add straight
    if ~isempty(find(N(:,1)==n0(1) & N(:,2)==n0(2) & N(:,3)==n0(3)))
        %node n0 is already in node list
        n0_ind =find(N(:,1)==n0(1) & N(:,2)==n0(2) & N(:,3)==n0(3));
    elseif ~isempty(find(N(:,1)==n0(1) & N(:,2)==n0(2)))
        %node n0 coordinates are found but not orientation
        N = [N; n0];
        n0_ind = size(N,1);
        
        %if orientation isn't in N, find two closest, then remove
        %old connection between these two
       
       [first_connection, second_connection]=same_location_successors(n0,N);

       [i_matrix,j_matrix,c_matrix]=find(A);
       
       %remove old connection
       ind1 = find(i_matrix == first_connection & j_matrix == second_connection);
       i_matrix(ind1)=[];
       j_matrix(ind1)=[];
       c_matrix(ind1)=[];
       ind2 = find(j_matrix == first_connection & i_matrix == second_connection);
       i_matrix(ind2)=[];
       j_matrix(ind2)=[];
       c_matrix(ind2)=[];
       
       %add new connection
       i_matrix= [i_matrix', n0_ind, n0_ind,first_connection, second_connection];
       j_matrix= [j_matrix', first_connection, second_connection,n0_ind, n0_ind]; 
       
       cost1 = edge_cost(n0, N(first_connection,:));
       cost2 = edge_cost(n0, N(second_connection,:));
       c_matrix=[c_matrix', cost1, cost2, cost1, cost2];
       
       A= sparse(i_matrix, j_matrix,c_matrix);
    else
        %node n0 is not on map...
    end
    
    if ~isempty(find(N(:,1)==ng(1) & N(:,2)==ng(2) & N(:,3)==ng(3)))
        %
        ng_ind =find(N(:,1)==ng(1) & N(:,2)==ng(2) & N(:,3)==ng(3));
    elseif ~isempty(find(N(:,1)==ng(1) & N(:,2)==ng(2)))
        %ng coordinates found
        N = [N; ng];
        ng_ind = size(N,1);
         
       [first_connection, second_connection]=same_location_successors(ng,N);
       [i_matrix,j_matrix,c_matrix]=find(A);
       
       ind1 = find(i_matrix == first_connection & j_matrix == second_connection);
       i_matrix(ind1)=[];
       j_matrix(ind1)=[];
       c_matrix(ind1)=[];
       ind2 = find(j_matrix == first_connection & i_matrix == second_connection);
       i_matrix(ind2)=[];
       j_matrix(ind2)=[];
       c_matrix(ind2)=[];
       
       
       [i_matrix,j_matrix,c_matrix]=find(A);
       i_matrix= [i_matrix', ng_ind, ng_ind,...
           first_connection, second_connection];
       j_matrix= [j_matrix', first_connection, second_connection,...
           ng_ind, ng_ind]; 
       
       cost1 = edge_cost(ng, N(first_connection,:));
       cost2 = edge_cost(ng, N(second_connection,:));
       c_matrix=[c_matrix', cost1, cost2, cost1, cost2];
       
       A= sparse(i_matrix, j_matrix,c_matrix);
    else
        %node ng not in map
    end
    
    
 %% Dijkstra
    route = [];
    Dead = []; %dead nodes
    costn0 = 0;
    Active = [];%Active includes the indices of active nodes
    Active = [n0_ind]; %one row includes the index of node in N
   
        
    cost_matrix = [];
    %Initialize cost matrix
    %add infinite cost for each node and 0 as the previous node index 
    for i=1:size(N,1)
         cost_matrix =[cost_matrix; i, Inf ,0];
    end
    cost_matrix(n0_ind, :) = [n0_ind,costn0,0];
        
    [i_matrix,j_matrix,edgecosts]=find(A);
    
    %searching for path until ending point is found
    while (true)        
        %find the smallest cost within active
        
        [minimum,index]= min(cost_matrix(Active,2)); %finds the index of 
        %the smallest cost within Active matrix
        min_node = Active(index); %converts the minimum Active matrix cost
        %index into node index
            
        nc = min_node; %nc is the node in Active with smallest cost
       
        
        
        indices_i = find(i_matrix==nc); %find all nc's from the startpoints
        indices_j = find(j_matrix==nc);
        Succ_ij=j_matrix(indices_i); %with previous indices, find  all endpoints
        Succ_ji= i_matrix(indices_j);
        Succ = union(Succ_ij,Succ_ji);
        
        %connected points are the successors
            
        
        %if we find successors (which we should)
        if size(Succ,1)>0
        
            %checking successors
            for i=1:size(Succ,1)
                n = Succ(i,:);
                
                index = find(i_matrix==nc & j_matrix==n); %find index for edge nc to n
                if isempty(index)
                   index = find(j_matrix==nc & i_matrix==n);
                end
                
                edgecost = edgecosts(index);
                
                AD = [Active;Dead]; 
    
                %not in active nor dead:
                if  isempty(find(AD==n))            
                    newcost = cost_matrix(nc,2) + edgecost;
                    Active = [Active;n];
                    cost_matrix(n,:)=[n,newcost,nc];
                %already in active
                elseif ~isempty(find(Active==n))
                    cost = cost_matrix(n,2);
                    newcost = cost_matrix(nc,2) + edgecost;
                    if newcost < cost
                        cost_matrix(n,:)=[n,newcost,nc];
                    end
                end
            end
        end
        
       
        %delete current one from active.
        ind = find(Active==nc);
        Active(ind)= [];
        
        Dead=[Dead; nc]; % Move nc  from Active and put it in Dead
        if nc == ng_ind
           min_cost = cost_matrix(nc,2);
           route_found=1;
           break 
        elseif isempty(Active)
            %route was not possible,
            %must add more nodes
            route_found = 0;
             break
        end
           
    end      
      if route_found ~=0
        %route can be calculated from cost_matrix by going from the goal to
        %the end
        oldprevious = ng_ind; %where we came from
        route=[ng_ind];
        while(oldprevious ~= n0_ind)
            
            newprevious = cost_matrix(oldprevious,3);
            route=[newprevious,route];
            oldprevious = newprevious;
        end
      
      
      figure
      plot(N(route, 1),N(route, 2),'r*-')
      title('Dijkstra algorithm')
      end
end

