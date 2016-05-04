function [dx,u] = pendulum_sys(t,x)
    g = 9.8;
    b = 0;
    l = 1;
    m = 1;

    % linearized dynamics about x
    A = [0 1; g/l -b/(m*l*l)];
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
