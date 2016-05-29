function [dx,path_cost] = linearized_pendulum_sys(t,x,x0,A,B,K)
    u = -K*x;

%{
    if(u > 3)
        u = 3;
    elseif(u < -3)
        u = -3;
    end
%}

    dx = A*x + B*u;

end
