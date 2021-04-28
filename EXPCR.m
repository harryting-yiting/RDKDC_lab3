function exp_x = EXPCR(xi,th)
% Return the 3 × 3 rotation matrix which represents a rigid-body rotation
    if th == 0
        exp_x = eye(3);
        return;
    end
    exp_x = eye(3)+ SKEW3(xi)*sin(th) + SKEW3(xi)* SKEW3(xi)*(1-cos(th));
end 
