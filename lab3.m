%% Lab3: Please run this file section by section!!

clear;
clc;
close all;

ur5 = ur5_interface();

% There is the offset rotation between ee_link and tool0. We first rotate
% the tool0 to ee_link
g_ee_tool0 = zeros(4,4);
g_ee_tool0(1:3,1:3) = eye(3);
g_ee_tool0(4,4) = 1;
move_frame('tool0','ee_link', g_ee_tool0);

%% Test cases: ur5FwdKin


%redefine base frame position based off of construction
%the ur5 configuration from assignment 4 number 2 is defined as gst0
%tf_frame('base_link', 'base', [ROTZ(pi/2) [0 0 0]'; 0 0 0 1]);
%pause(1)

% Part 3 a) Forward Kinematic Map Verification

fprintf('\n\n-------Beginning testing of ur5FwdKin()-----------\n')
%% ur5FwdKin: case 1
q1 = [0, pi/4, -pi/2, 0, pi/2, 0]';
g1 = ur5FwdKin(q1);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',g1);
ur5.move_joints(q1, 7);
pause(8)
std1 =  ur5.get_current_transformation('base_link','ee_link');
err1 = norm(std1- g1);
fprintf('\terror between current position and forward map is %d\n', err1);

%% ur5FwdKin: case 2
q1 = [pi/2, -pi/3, pi/3, pi/3, pi/2, pi/2]';
g1 = ur5FwdKin(q1);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',g1);
ur5.move_joints(q1, 7);
pause(8)
std1 =  ur5.get_current_transformation('base_link','ee_link');
err1 = norm(std1- g1);
fprintf('\terror between current position and forward map is %d\n', err1);

%% ur5FwdKin: case 3
q1 = [0, 0, 0, 0, 0, 0]';
g1 = ur5FwdKin(q1);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',g1);
ur5.move_joints(q1, 7);
pause(8)
std1 =  ur5.get_current_transformation('base_link','ee_link');
err1 = norm(std1- g1);
fprintf('\terror between current position and forward map is %d\n', err1);


fprintf('-------Finished testing of ur5FwdKin() ------------\n\n')

%% Test cases: ur5BodyJacobian
fprintf('-------Beginning testing of ur5BodyJacobian()---------\n')

for i = 1:10
    q = [rand(1,6)*2*pi - pi]';
    g = ur5FwdKin(q);
    JB = ur5BodyJacobian(q);
    Japprox = zeros(6,6);       %matrix for jacobian approximation
    e = eye(6);
    eps = 1e-8;
    for i = 1:6
        ei = e(:,i);    %get the current basis vector
        dgdqi = 1/2/eps * ( ur5FwdKin(q + eps*ei) - ur5FwdKin(q - eps*ei) );
        xi_ = FINV(g)*dgdqi;
        Japprox(:,i) = DESKEW4(xi_);

    end

    err = norm(JB - Japprox);
    fprintf('\terror between Jacobian and approximation is %d\n', err);
    
end
fprintf('-------Finished testing of ur5BodyJacobian()---------\n')
%% Test cases: Manipulability

fprintf('-------Beginning testing of manipulability()---------\n')
% theta_3 from -pi/2 tp pi/2 

% joint angles
q1 = pi/3;
q2 = 0;
q3_init = -pi/2;
q3_goal = pi/2;
q4 = pi/4;
q5 = pi/2;
q6 = pi/2;

% change theta3 from -pi/2 to pi/2
q3s = q3_init:0.05:q3_goal;

% value of different kinds of manipulability
sigmamin = zeros(length(q3s),1);
detjac =  zeros(length(q3s),1);
invcond = zeros(length(q3s),1);

i_mani = 0;
for q3 = q3s
    
    i_mani = i_mani + 1;
    q_test_manipu = [q1, q2, q3, q4, q5, q6]';
    JB = ur5BodyJacobian(q_test_manipu);
    
    % comp
    sigmamin(i_mani) = manipulability(JB, 'sigmamin');
    detjac(i_mani) = manipulability(JB, 'detjac');
    invcond(i_mani) = manipulability(JB, 'invcond');
    
end

% plot figure
figure
plot(q3s, sigmamin)
hold on
plot(q3s, detjac)
hold on
plot(q3s, invcond)
legend('sigmamin', 'detjac', 'invcond')
title('Manipulability for theta3 = [-pi/2, pi/2]')

fprintf('-------Finished testing of manipulability()---------\n')
%% Test cases: getXi
fprintf('-------Beginning testing of getXi()--------------\n')

% ***** One important point is that w must be an unit vector in order to
% use TwistExp
% Case 1

% compute homogenuous transformation matrix
xi = [2;0;0;1;0;0];
theta = pi/2;
g = TwistExp(xi,theta); 
fprintf('Homogenuous transformation function\n')
disp(g);
% getXi
xi_test = getXi(g);
fprintf('Original twist\n')
disp(xi*theta);
fprintf('Return twist\n')
disp(xi_test);
% compute error between original maxtirx and expm(xi)
xi_hat = [SKEW3(xi_test(4:end)),xi_test(1:3);zeros(1,4)];
g_test  = expm(xi_hat);
fprintf('Error between Resulted Transformation matrix and Original matrix\n')
disp(g_test-g)

%% getXi: Case 2

% compute homogenuous transformation matrix
xi = [1;1;1;0.5;0.5;sqrt(0.5)];
theta = pi/4;
g = TwistExp(xi,theta);
fprintf('Homogenuous transformation function\n')
disp(g);
% getXi
xi_test = getXi(g);
fprintf('Original twist\n')
disp(xi*theta);
fprintf('Return twist\n')
disp(xi_test);
% compute error between original maxtirx and expm(xi)
xi_hat = [SKEW3(xi_test(4:end)),xi_test(1:3);zeros(1,4)];
g_test  = expm(xi_hat);
fprintf('Error between Resulted Transformation matrix and Original matrix\n')
disp(g_test-g)

%% getXi: Case 3

% compute homogenuous transformation matrix
v = [-3;10;-0.2];
a = 0.9;
b = -0.2;
c = sqrt(1- a^2 - b^2);
xi = [v;a;b;c];
theta = -pi/4;
g = TwistExp(xi,theta);
fprintf('Homogenuous transformation function\n')
disp(g);
% getXi
xi_test = getXi(g);
fprintf('Original twist\n')
disp(xi*theta);
fprintf('Return twist\n')
disp(xi_test);
xi_hat = [SKEW3(xi_test(4:end)),xi_test(1:3);zeros(1,4)];
g_test  = expm(xi_hat);
fprintf('Error between Resulted Transformation matrix and Original matrix\n')
disp(g_test-g)

fprintf('-------Finished testing of manipulability()---------\n')
%% Test cases: control 1st - no singular
% Please run the test cases section by section!!

%% Initilize the pose of ur5 to a non-singular configuration
fprintf('-------Beginning testing of ur5RRcontrol() ---------\n')

q_control_init1 = [pi/3, 0, -pi/2, pi/2, pi/4, pi/2]';
g_control_init1 = ur5FwdKin(q_control_init1);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',g_control_init1);
ur5.move_joints(q_control_init1, 10);
pause(10)
%% Move ur5 to the goal configuration
q_control_goal1 = [pi/2, pi/3, -pi/2, 0, pi/3, pi/3]';
g_control_goal1 = ur5FwdKin(q_control_goal1);
goalFrame = tf_frame('base_link','goal',g_control_goal1);
K = 0.5;
dis = ur5RRcontrol(g_control_goal1, K, ur5);
disp('Positional error: ' + string(dis)+ 'CM');

%% Test cases: control 2ed - goal pose will be near singularity

%% Initilize the pose of ur5 to a non-singular configuration

q_control_init2 = [pi/3, 0, -pi/2, pi/2, pi/4, pi/2]';
g_control_init2 = ur5FwdKin(q_control_init2);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',g_control_init2);
ur5.move_joints(q_control_init2, 10);
pause(10)
%% Move ur5 to the goal configuration which is near singularity
q_control_goal2 = [pi/3, -pi/6, 0, -pi/6,0, 0]';
g_control_goal2 = ur5FwdKin(q_control_goal2);
goalFrame = tf_frame('base_link','goal',g_control_goal2);
K = 0.5;
dis = ur5RRcontrol(g_control_goal2, K, ur5)

fprintf('-------Finished testing of ur5RRcontrol() ---------\n')

