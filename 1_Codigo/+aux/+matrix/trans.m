function M = trans(x,y,z) % move em xyz
    M = eye(4);  M(1:3,4) = [x;y;z];
end