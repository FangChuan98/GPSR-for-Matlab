function dis2 = cal_dis(A,B)
    global node;
    %nodeA & nodeB has to be id
    dis2 = (node(A).x - node(B).x)^2 + (node(A).y - node(B).y)^2 + (node(A).z - node(B).z)^2; 

end

