function [dx,path_cost] = double_intg_sys(t,x,A,B,K)
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
