function [dx,path_cost] = linearized_pendulum_sys(t,x,x0,A,B,K)

    Ac = [(A-B*K)];

    dx = Ac*x;
end
