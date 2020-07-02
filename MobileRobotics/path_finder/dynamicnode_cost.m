%edge_cost_undirected.m successors.m


function [cost, recursion_ended,optimalcosts,visited] = dynamicnode_cost(optimalcosts,ng,node, k, E,L, recursion_ended,visited)
    
    visited=[visited;node];


    if  k > 1 & recursion_ended == 0
        Succ=successors(E,node);
        
        %delete visited from successors
        Succ = setdiff(Succ,visited);%returns the data in Succ that is not in visited
        
        if ~isempty(Succ)
        
       
            %go through all successors
            for ns_ind=1:size(Succ,1)
                ns=Succ(ns_ind);
            
                % end recursion if goal is found
               if ns == ng
                    cost_ng_ns = optimalcosts(ng);
                    recursion_ended=1;
               else
                
                    [cost_ng_ns,recursion_ended,optimalcosts,visited]= ...
                        dynamicnode_cost(optimalcosts,ng,ns,k-1,E,L, ...
                        recursion_ended,visited);
               end      
            
               new_cost=edge_cost_undirected(node,ns,L)+cost_ng_ns;
            
                %if we find a cost that is smaller than previous, that is
                %the new optimal value
                if new_cost < optimalcosts(node)
                    optimalcosts(node)= new_cost;
                end
            end
        else
            recursion_ended=1;
        end
    else
        recursion_ended=1;
        
    end
    
    cost = optimalcosts(node);
    
end