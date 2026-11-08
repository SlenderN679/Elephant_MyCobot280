function M = rotx(t)      % roda em x
    M = [1 0 0 0; 
        0 cos(t) -sin(t) 0; 
        0 sin(t) cos(t) 0; 
        0 0 0 1];
end