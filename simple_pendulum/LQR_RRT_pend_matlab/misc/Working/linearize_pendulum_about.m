function [A,B] = linearize_pendulum_about(x)
    g = 9.8;    % acceleration due to gravity
    b = 0.1;    % damping
    l = 1;      % length of pendulum arm
    m = 1;      % mass at end of pendulum arm

    % linearized dynamics about state x
    A = [0 1; (g*sin(x(1)))/l -b/(m*l*l)];  % x(1) is the angular position measured counter-clockwise from horizontal plane
    B = [0; 1/(m*l*l)];

end
