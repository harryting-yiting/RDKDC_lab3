%% Task 1.1, Lab3
clear;
clc;
close all;

% initialize
ur5 = ur5_interface();

% first case
q1 = [0, pi/4, -pi/2, 0, pi/2, 0]';
ur5.move_joints(q1, 7);
pause(10)
theta1 = ur5.get_current_joints();
error1 = q1- theta1;

disp('The differences between target joint angles and real final angles ara: ')
error1
% second case

q2 = [pi/2, -pi/3, pi/3, pi/3, pi/2, pi/2]';
ur5.move_joints(q2, 7);
pause(10)
theta2 = ur5.get_current_joints();
error2 = q2- theta2;

disp('The differences between target joint angles and real final angles ara: ')
error2
% Third case

q3 = [pi/3, 0, -pi/2, pi/2, pi/4, pi/2]';
ur5.move_joints(q3, 7);
pause(10)
theta3 = ur5.get_current_joints();
error3 = q3- theta3;

disp('The differences between target joint angles and real final angles ara: ')
error3
% final case

q4 = [0, 0, 0, 0, 0, 0]';
ur5.move_joints(q4, 7);
pause(10)
theta4 = ur5.get_current_joints();
error4 = q4- theta4;

disp('The differences between target joint angles and real final angles ara: ')
error4