function no_coll = multiple_collision(n2, n1, o)
    
    % Size of object matrix
    s = size(o);
    % Number of object
    for i=1:s(1)
        % Singe collision
        ncs(i) = noCollision(n2, n1, o(i,:));
    end
    
    % Return "true" if there aren't collision 
    if(sum(ncs)==s(1))
        no_coll=1;
    else 
        no_coll=0;
    end

end