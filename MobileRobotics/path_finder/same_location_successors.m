%needs file orientation_min2max.m


function [first_connection, second_connection, Ind] = same_location_successors(n,N)
        %finds nodes that are closest to n's orientation in both sides.
        
        %organize indices (orientations from smallest to highest in the
        %location
       Ind = orientation_min2max(n,N);
       orientation = n(:,3);
       
       %start from the first index, initializing
       minimumdistance = abs(angdiff(N(Ind(1),3),orientation));
       min_ind_Ind = 1;
      
        %go through the rest of the nodes in location
       for index=2:size(Ind,1)
           
           %save the closest orientation (the index of that node in Ind)
         if minimumdistance > abs(angdiff(N(Ind(index),3),orientation)) && ...
                 abs(angdiff(N(Ind(index),3),orientation)) ~=0
             minimumdistance = abs(angdiff(N(Ind(index),3),orientation));
             min_ind_Ind = index;
         end
       end
       
       %save the N matrix index of the first connection (closest orientation)
       first_connection = Ind(min_ind_Ind);
       %orientations' indices are in order from -pi to pi in Ind matrix
       %therefore node n is before or after first_connection in that
       %matrix and second connection must also be before or after n.
       n_ind = find(N(:,1) == n(1) & N(:,2)==n(2) & N(:,3)==n(3));
       n_ind_Ind = find(Ind == n_ind)
       
       if N(first_connection,3) < orientation 
            
            if min_ind_Ind == 1 & n_ind_Ind == size(Ind,1)
                %when n is the max and first_connection min
                second_connection = Ind(size(Ind,1)-1);
            else
                second_connection = Ind(min_ind_Ind+2);
            end
       else
           if min_ind_Ind == size(Ind,1) & n_ind_Ind == 1
               %when first_connection is the max and n min
                second_connection = Ind(2) ;
           else
                second_connection = Ind(min_ind_Ind-2);
           end
       end
end