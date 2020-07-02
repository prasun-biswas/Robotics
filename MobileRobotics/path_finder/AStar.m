%% A* algorithm
function [path_found_flag, path_result] = AStar(L, node_edge_map)
    startNode = 1; % the start point is the startNode in the L-matrix(node coordinate matrix) 
    endNode = 102; % the end point is the endNode in the L-matrix(node coordinate matrix) 
    activeNodes = startNode; %activeNodes matrix is initialised
    deadNodes = 0; %deadNodes matrix is initialised with 0 which isnot valid node number (node numbers are positive)
    previousDeadNodes = 0;  %previousDeadNodes matrix is initialised with 0 
                            % which isnot valid node number (node numbers are 
                            % positive) - this matrix keeps the nodes that do
                            % not have any valid successors
    lowAppCostNode = startNode; %node with the lowest cost to reach from curren
                                %t node to next node
    costToGo = [startNode,0]; % this matrix keeps the nodes and their edge cost and orientation cost
    heuristic_multiplier = 1; % heuristic weight 
    lowAppCost = [startNode, heuristic_multiplier*pdist2(L(startNode,:), L(endNode,:), 'euclidean')]; % this matrix keep the nodes and their cost to reach
    path = []; %empty path defined at start
    while 1
        path = [path,lowAppCostNode]; % path inserted with lowAppCostNode for each iteration
        succ = node_edge_map(node_edge_map(:,1) == lowAppCostNode,2); % get successor of the current node 
            % search among the successor node the node with the lowest cost to
            % reach
            for node = 1:size(succ,1)
                    % check the successor is already in active or dead
                    if (~(any(activeNodes(1,:)==succ(node))) && (~(any(deadNodes(1,:)==succ(node)))))
                        activeNodes = [activeNodes,succ(node)]; % add successor as active 
                        % get edge cost and orientation cost of current node  
                        costToNode = costToGo(costToGo(:,1)==lowAppCostNode,2)+ pdist2(L(succ(node),:),L(lowAppCostNode,:),'euclidean') + atan((L(lowAppCostNode,2)-L(succ(node),2))/(L(lowAppCostNode,1)-L(succ(node),1)));
                        % add to the costToGo matrix with node and cost
                        costToGo = [costToGo;[succ(node), costToNode]];
                        % add the heuristic cost
                        lowCostToNode = costToGo(costToGo(:,1)==succ(node),2) + heuristic_multiplier*pdist2(L(succ(node),:), L(endNode,:), 'euclidean');
                        % add to the lowAppCost matrix 
                        lowAppCost = [lowAppCost; [succ(node), lowCostToNode]];
                    elseif any(activeNodes(1,:)==succ(node)) 
                        % if already present is activeNodes update the cost
                        % value
                        edgeCostToNode = pdist2(L(succ(node),:), L(lowAppCostNode,:), 'euclidean') + atan((L(lowAppCostNode,2)-L(succ(node),2))/(L(lowAppCostNode,1)-L(succ(node),1)));
                        costToGo(costToGo(:,1) == succ(node),2) = min(costToGo(costToGo(:,1) == succ(node),2), costToGo(costToGo(:,1) == lowAppCostNode,2)+edgeCostToNode);
                        lowCostToNode = costToGo(costToGo(:,1) == succ(node),2) + heuristic_multiplier*pdist2(L(succ(node),:), L(endNode,:), 'euclidean');
                        lowAppCost(lowAppCost(:,1)==succ(node),2) = lowCostToNode; 
                    end
            end
            % remove the current node from active nodes
            activeNodes(activeNodes(1,:) == lowAppCostNode) = [];
            % add the current node to dead Node
            if(~any(deadNodes(1,:)==lowAppCostNode))
                deadNodes = [deadNodes,lowAppCostNode];
            end        
            % check whether goal is reached
            if(lowAppCostNode == endNode)
                path_plot = zeros(size(path(1,:),2),2);
                for node = 1:size(path(1,:),2)
                    path_plot(node,1) = L(path(node),1);
                    path_plot(node,2) = L(path(node),2);
                end
                plot(path_plot(:,1),path_plot(:,2),'r*-')
                path_found_flag = 1;
                path_result = path_plot;
                break
            else
                tempLowCostToGo = [];
                % check if all the successor are in dead Node
                if all(ismember(succ,deadNodes))
                    if(lowAppCostNode == 1) %if startNode has no valid successor then there is no path
                        disp('NO PATH FOUND')
                        path_found_flag = 0;
                        path_result = [];
                        break
                    end
                    % store the dead nodes from previous iteration
                    previousDeadNodes = [previousDeadNodes, lowAppCostNode];
                    deadNodes = 0; % reinitialise deadNodes
                    deadNodes = [deadNodes,previousDeadNodes]; % initialise with previous dead Nodes
                    activeNodes = startNode; % reinitialise activenode as startNode
                    lowAppCostNode = startNode; % reinitialise lowAppCostNode
                    costToGo = [startNode,0]; %reinitialise
                    lowAppCost = [startNode, heuristic_multiplier*pdist2(L(startNode,:), L(endNode,:), 'euclidean')];%reinitialise
                    path = [];%reinitialise
                    continue %skip the rest of codes
                end
                succDeadNodes = intersect(deadNodes,succ); %check successors already in deadNodes
                expansionNodes = setxor(succ,succDeadNodes); % remove those successors
                % store the cost of valid successors in tempLowCostToGo
                for node = 1:size(expansionNodes,1)
                    tempLowCostToGo = [tempLowCostToGo;lowAppCost(lowAppCost(:,1)==expansionNodes(node),:)];
                end
                % find the node with minimum cost and select it for next
                % iteration
                lowAppCostNode = tempLowCostToGo(tempLowCostToGo(:,2) == min(tempLowCostToGo(:,2)));
            end
    end
end

