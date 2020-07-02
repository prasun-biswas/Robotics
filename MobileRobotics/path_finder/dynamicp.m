%needs files dynamicnode_cost.m, successors.m, edge_cost_undirected.m

function [route, route_is_found]=dynamicp(ng,n0,L,E)
    
    ng_ind=find(L(:,1)==ng(1) & L(:,2)==ng(2));
    
    %% costmatrix
    %J(n,ng) >0 (Inf)
    optimalcosts =zeros(size(L,1),1)+Inf; % one row = J(index,ng)
    
    % J(ng,ng) = 0;
    optimalcosts(ng_ind)=0;
   
    
    %% recursion for all n's
    
    %Iterate for all n: 
    %J(n,ng)=min|edgecost(node,ns)+ J(ns,ng)|
    
    
%cost from n to ng
   for n=1:size(L,1)
     
        recursion_ended = 0;
        k= size(L,1)-1;
        visited = [];
        [cost,recursion_ended,optimalcosts,visited]= dynamicnode_cost(optimalcosts,ng_ind,n,k,E,L,recursion_ended,visited);
        
    end    
 
    
    %% path
%path is optimal,so if we want a route from n0 we go to successor of n0
%that has lowest cost (to go to ng) etc
    n0_ind=find(L(:,1)==n0(1) & L(:,2)==n0(2));
    route = [n0_ind];
    n= n0_ind;
%only if cost to n0_ind is not Inf there is a path to n0.
    if optimalcosts(n0_ind)< Inf
        while(n~=ng_ind)
            S=successors(E,n);
            [minimum,index] = min(optimalcosts(S));
            nextnode=S(index);
            route =[route,nextnode];
            n=nextnode;
        end
        figure
        plot(L(route, 1),L(route, 2),'r*-')
        title('Dynamic programming algorithm')
        route_is_found = 1;
    else
        %route wasn't found
        route_is_found = 0;
    end
    
end


