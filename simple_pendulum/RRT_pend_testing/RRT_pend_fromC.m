m = 1;		% mass
g = 9.8;	% acceleration due to gravity
l = 1;	    % length of pendulum link
b = 0.1;	% damping factor
theta_0 = -pi/2;    % initial angular position
dt = 0.02;			% time interval between application of subsequent control torques

M = importdata('../RRT_pend_C/build-RRT_pend_C-Desktop_Qt_5_3_GCC_64bit-Debug/data.txt');
control = [[0:dt:dt*length(M)-dt]',M];
