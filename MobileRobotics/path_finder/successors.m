function S = successors(E,n)
    
    %find all connections n has in E
    indices_1 = find(E(:,1)==n);%find n's from startpoints
    indices_2 = find(E(:,2)==n);%find n's from end points
    S=union(E(indices_1,2), E(indices_2,1)); %successors are either the endpoints
    %with n as start point or start points with n as end point
end