function [dx,path_cost] = 2D_double_integrator_sys(t,x,K)
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
