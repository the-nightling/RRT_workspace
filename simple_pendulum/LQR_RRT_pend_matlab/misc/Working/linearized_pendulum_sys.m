function [dx,path_cost] = linearized_pendulum_sys(t,x,x0,A,B,K)

%    u = -K*x;
    Ac = [(A-B*K)];

    dx = Ac*x;
end
