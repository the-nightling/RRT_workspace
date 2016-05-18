function [A,B] = linearize_pendulum_about(x)
    
    [g,b,l,m] = set_pendulum_parameters();
    
    % linearized dynamics about state x
    A = [0 1; (g*sin(x(1)))/l -b/(m*l*l)];  % x(1) is the angular position measured counter-clockwise from horizontal plane
    B = [0; 1/(m*l*l)];

end
