function Ind=orientation_min2max(n,N)
    %function organizes the nodes in the same location from smallest
    %orientation to highest. Orientation is within range -pi to pi
    
    %find location from node matrix N
    indices = find(N(:,1)==n(1) & N(:,2)==n(2));
    N_n = N(indices,3); %orientations of the same coordinate nodes
       
    Ind = []; %initialize
        
    %orientations are within range -pi to pi
    %we go through N_n matrix
    %and list the indices in N for orientations from smallest to highest value
    %and save them in Ind
    while(size(N_n,2)> 0)
        [minim,i_N_n] = min(N_n); %minimum value and 
         %the index that is going to be removed from N_n
        i_min1=find(N(indices,3)==minim); %index in indices with minimum 
        i_min2 = indices(i_min1); %index in N with minimum 
         %orientation (same coordinates)

        Ind = [Ind ; i_min2]; %add index in list that organizes 
        
        %chosen nodes from smallest orientation to highest
        N_n(i_N_n) = []; %when checked, remove from N_n
    end
end