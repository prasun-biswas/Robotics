%needs files edge_cost.m and orientation_min2max.m

function [N,A]=convert2digraph(L,E)
    %L includes node coordinates
    %E includes edges 
    
    %% N matrix from nodes
    N=[];
    %N includes all nodes that are connected to another
    %E matrix row by row (from node to node)
    for j=1:size(E,1)       
           
           ni=L(E(j,1),:); %start node
           nj=L(E(j,2),:);%end node
           
           ninj= [ni(1)-nj(1),ni(2)-nj(2)]; %vector from node i to j
           
           %orientation is the angle between positive x-axis and the edge
           %from i to j
           %clockwise negative
           orientation = acos(dot(ninj,[1,0])/(norm(ninj)*norm([1,0])));
           
           %checking the orientation
           %as angle is calculated between edge and positive x-axis:
           if(ninj(1)>=0 & ninj(2)>=0)
               orientation = orientation;
           elseif  (ninj(1)<0 & ninj(2)<0)   
               orientation = (-1)*orientation;
           elseif (ninj(1)>=0 & ninj(2)<0)
               orientation = (-1)*orientation;
           else
               orientation= orientation;
           end    
           
           %if there is an edge, there must be two orientations 
           %to go from point j to i and i to j
           %these orientations are opposite
           
           %orientation and opposite orientation for both nodes
           if orientation >=0
               N = [N;L(E(j,1),:),orientation-pi];%start node orientation 1
               N = [N;L(E(j,1),:),orientation];%start node orientation 2
               N = [N;L(E(j,2),:),orientation-pi]; %end node orientation 1
               N = [N;L(E(j,2),:),orientation];%end node orientation 2
               
               %therefore N(i,:) and N(i+2,:) are will be connected 
               %and N(i+1,:)and N(i+3,:) will be connected
           else
               %for orientation to stay between -pi to pi
               N = [N;L(E(j,1),:),orientation+pi];
               N = [N;L(E(j,1),:),orientation];
               N = [N;L(E(j,2),:),orientation+pi];
               N = [N;L(E(j,2),:),orientation];
           end
           
     end
        
     %% generate adjacency matrix A
     
    i_matrix =[]; %node indices (N matrix) for the start nodes
    j_matrix =[]; %node indices (N matrix) for the end nodes
    cost_matrix= []; %costs between node i and j. 
    %(1st cost is the cost between 1st i and 1st j etc) 
    
    
    % add edges between two locations
    i=1;
    while( i < size(N,1))        
        %N(i,:) and N(i+2,:) connected and N(i+1,:)and N(i+3,:)connected
        %because of the formulation of N above
        %one has to face from location 1 to location 2 and other from
        %location 2 to location 1
        i_matrix = [i_matrix,i,i+3];
        j_matrix = [j_matrix,i+2,i+1];
        cost0213 = edge_cost(N(i,:),N(i+2,:));
        cost_matrix= [cost_matrix,cost0213,cost0213];
        i=i+4;
    end
    
    %add edges within the same location
    %N find all rows for one coordinate and make a new matrix
    for j=1:size(L,1)
        N_index = find(N(:,1)==L(j,1) & N(:,2)==L(j,2)); %all N:s with L coordinates
   
        
        
        %only continuing if L is found from N (so L is actually connected to
        %another and that way has orientation)
        if isempty(N_index) ~= 1
                   
          
            Ind = orientation_min2max(L(j,:),N);
            
        
            %connect minimum orientations to each other one by one and finally
            %the max orientation to the min (orientations from -pi to pi)
            for jj=1:size(Ind,1)-1
                if jj==1
                    %minimum and maximum orientation
                    if jj==1 & size(Ind,1) ~=2
                        i_matrix = [i_matrix,Ind(jj),...
                            Ind(size(Ind,1)),...
                            Ind(jj+1),Ind(jj)];
                        j_matrix = [j_matrix,Ind(size(Ind,1)),...
                            Ind(jj),...
                            Ind(jj),Ind(jj+1)];
             
                        cost = edge_cost(N(Ind(jj),:),...
                            N(Ind(size(Ind,1)),:));
                        cost2 = edge_cost(N(Ind(jj),:),...
                            N(Ind(jj+1),:));
                        cost_matrix =[cost_matrix, cost,cost, cost2,cost2];
                    elseif jj==1 & size(Ind,1) ==2
                        %only two orientations for the node
                        i_matrix = [i_matrix,Ind(jj),...
                            Ind(size(Ind,1))];
                        j_matrix = [j_matrix,Ind(size(Ind,1)),...
                            Ind(jj)];
        
                        cost = edge_cost(N(Ind(jj),:),...
                            N(Ind(size(Ind,1)),:));
                        cost_matrix =[cost_matrix, cost,cost];
                    
                    end
                else
                    
                    % N_index(Ind(jj))
                    %N_index(Ind(jj+1))
                    i_matrix = [i_matrix,Ind(jj),Ind(jj+1)];
                    j_matrix = [j_matrix,Ind(jj+1),Ind(jj)];
                
                    cost = edge_cost(N(Ind(jj),:),...
                        N(Ind(jj+1),:));
                    cost_matrix =[cost_matrix, cost,cost];
                end             
            end
        end
    end
    
    %create A matrix
    A=sparse(i_matrix,j_matrix,cost_matrix);

end