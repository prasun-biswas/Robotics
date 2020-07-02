function cost = edge_cost_undirected(i1,i2,L)
    
    n1= L(i1,:);
    n2 =L(i2,:);

    nanb= [abs( n1(1)-n2(1)),abs(n1(2)-n2(2))]; %vector between nodes
    c= 1;  %insert a value
    
    %distance = norm(na-nb);%...or
    distance = norm(nanb);%distance between two points na and nb
    
    %returns the cost 
    cost= c*(distance);

end