function xi = getXi(g)
% Input: g       -> Homogeneous transformation matrix [4,4]
%      
% Output: xi     -> Corresponding unscaled matrix  

if size(g) ~= [4,4]
    error('Error. \n Dimension of Transformation Mtarix is not [4,4].');
end

R = g(1:3,1:3);
theta = acos((trace(R) - 1)/2);

if theta == 0 || theta == pi
     fprintf('Error. \n Singular value.');
     return;
end

w = [R(3,2) - R(2,3);
     R(1,3) - R(3,1);
     R(2,1) - R(1,2)]./(2*sin(theta));
A = (eye(3) - R)*SKEW3(w) + w*w.'*theta;
v = inv(A)*g(1:3,4);
xi = [v;w]*theta;




end

function skew = SKEW3(x)
% accepts a 3×1 vector x = [x1,x2,x3]T and returns the corresponding 
% canonical 3 × 3 skew-symmetric matrix
    skew = [0, -x(3), x(2);...
            x(3), 0 , -x(1);...
            -x(2), x(1), 0];
end 