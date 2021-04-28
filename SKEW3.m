function skew = SKEW3(x)
% accepts a 3×1 vector x = [x1,x2,x3]T and returns the corresponding 
% canonical 3 × 3 skew-symmetric matrix
    skew = [0, -x(3), x(2);...
            x(3), 0 , -x(1);...
            -x(2), x(1), 0];
end 
