function exp_x = TwistExp(xi,th)
    w = xi(4:end);
    v = xi(1:3);
    R = EXPCR(w,th);
    p = (eye(3) - R) * (SKEW3(w)*v) + w * (w.' * v) * th;
    exp_x =[R,p;[0,0,0,1]];
end 
