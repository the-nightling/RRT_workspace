function dx = pendulum_m(t, x)
%PENDULUM_M  A pendulum system.
    
    g = 9.8;
    l = 1;
    b = 0.2;
    m = 1;
    

    % State equations.
    dx = [x(2);                             ... % Angular position.
        -(g/l)*sin(x(1))-b/(m*l^2)*x(2)   ... % Angular velocity.
       ];
end
