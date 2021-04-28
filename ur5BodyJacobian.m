%Q = [0 0 0 0 0 0];
%r5BodyJacobian(Q)
function JB = ur5BodyJacobian(Q)
%Q =[0 0 0 0 0 0];

% Defintions
L0 = 0.0892;
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

th1 = Q(1);
th2 = Q(2);
th3 = Q(3);
th4 = Q(4);
th5 = Q(5);
th6 = Q(6);
xi1 = zeros(6,1); xi1(6) = 1;
xi2 = zeros(6,1); xi2(4) = 1; xi2(2) = L0;
xi3 = zeros(6,1); xi3(4) = 1; xi3(2) = L0+L1;
xi4 = zeros(6,1); xi4(4) = 1; xi4(2) = L0+L1+L2;
xi5 = zeros(6,1); xi5(6) = 1; xi5(2) = -L3;
xi6 = zeros(6,1); xi6(4) = 1; xi6(2) = L0+L1+L2+L4;
xi = [xi1,xi2,xi3,xi4,xi5,xi6];
E1 =TwistExp(xi1,th1);
E2 =TwistExp(xi2,th2);
E3 =TwistExp(xi3,th3);
E4 =TwistExp(xi4,th4);
E5 =TwistExp(xi5,th5);
E6 =TwistExp(xi6,th6);

gst0 = [eye(3),[L3+L5,0,L0+L1+L2+L4].';0,0,0,1];

gst = E1*E2*E3*E4*E5*E6*gst0;

JS = zeros(6,6);
JB = zeros(6,6);
for i = 1:6
    tmp = eye(4);
    for j = 1:i-1
        tmp = tmp * TwistExp(xi(:,j) , Q(j));
    end
    JS(:,i) = adj(tmp)*xi(:,i);
end
JB = adjinv(gst)*JS;


end
function mat = adjinv(g)

R = g(1:3,1:3);
t = g(1:3,4);

mat = [
    R', -SKEW3(R'*t)*R'; 
    zeros(3,3), R';
    ];
end
function mat = adj(g)

R = g(1:3,1:3);
t = g(1:3,4);

mat = [
    R, SKEW3(t)*R; 
    zeros(3,3), R;
    ];
end
