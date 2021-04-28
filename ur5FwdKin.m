function gst = ur5FwdKin(Q)

L0 = 0.0892;
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = .0825;
%ur5
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

E1 =TwistExp(xi1,th1);
E2 =TwistExp(xi2,th2);
E3 =TwistExp(xi3,th3);
E4 =TwistExp(xi4,th4);
E5 =TwistExp(xi5,th5);
E6 =TwistExp(xi6,th6);

gst0 = [eye(3),[L3+L5,0,L0+L1+L2+L4].';0,0,0,1];

gst = E1*E2*E3*E4*E5*E6*gst0;

end