function [dx,path_cost] = linearized_pendulum_sys(t,x,x0,A,B,K)
    u = -K*x;
    %%{
       if(u > 3)
            u = 3;
       elseif(u < -3)
        u = -3;
       end
%}
    u
    dx = A*x + B *u;

%    Ac = [(A-B*K)];

%    dx = Ac*x;
end
