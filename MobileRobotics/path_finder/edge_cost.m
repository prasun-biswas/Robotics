function cost = edge_cost(n1,n2)
    %calculates the cost between two nodes n1 and n2
    %n1 and n2  are nodes: n=[direction x, direction y, orientation rad]
    
    nanb= [abs( n1(1)-n2(1)),abs(n1(2)-n2(2))]; %vector between nodes
    c= 1;  %insert a value
    
    %distance = norm(na-nb);%...or
    distance = norm(nanb);%distance between two points na and nb
    
    %calculate shortest rotation
    radians= abs(angdiff(n2(:,3),n1(:,3)));
       
    %returns the cost 
    cost= c*(distance+radians);
end
