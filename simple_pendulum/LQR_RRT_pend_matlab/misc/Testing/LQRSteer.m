function [t,y] = LQRSteer(x_nearest,x_rand)
    t_span = [0:0.04:20];
%    x_nearest = [-pi;0];

    [t,y] = ode45(@(t,y) pendulum_sys(t,y,x_rand), t_span, x_nearest, odeset('Events',@eventfun));
    
    
    plot(y(:,1),y(:,2));
end

function [value,isterminal,direction] = eventfun(t,y)
    threshold = 0.1;
    value      = double((y(1)*y(1) + y(2)*y(2)) < (threshold*threshold));
    isterminal = 1;
    direction  = 0;
end
