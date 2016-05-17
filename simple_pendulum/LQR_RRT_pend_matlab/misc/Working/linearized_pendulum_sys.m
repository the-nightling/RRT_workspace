function dx = linearized_pendulum_sys(t,x,x0, Q, R)

    % linearize non-linear pendulum about x0
    [A,B] = linearize_pendulum_about(x0);
    C = [1 0];
    D = [0];

    % apply LQR control


    [K,S] = lqr(A,B,Q,R);

%    u = -K*x;
    Ac = [(A-B*K)];
%    size(Ac)
%    size(x)
    dx = Ac*x;
end
