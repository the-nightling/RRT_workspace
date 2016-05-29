function [dx,u] = pendulum_sys(t,x,x0)
    g = 9.8;
    b = 0;
    l = 1;
    m = 1;

    % linearized dynamics about phi = [pi/2 0]
%    x0 = [pi/2 0];
%    A = [0 1; g/l -b/(m*l*l)];
    A = [0 1; (g*sin(x0(1)))/l -b/(m*l*l)];
    B = [0; 1/(m*l*l)];
    C = [1 0];
    D = [0];

    Q = diag([1 10]);
    R = 1;

    [K,S] = lqr(A,B,Q,R);

    u = -K*x;
    Ac = [(A-B*K)];

    dx = Ac*x;
end
