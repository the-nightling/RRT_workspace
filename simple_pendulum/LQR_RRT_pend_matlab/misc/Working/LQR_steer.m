function [t,y] = LQRSteer(x_nearest,x_rand)
    time_span = [0:0.04:10];
%    x_nearest = [-pi;0];

    [t,y] = ode45(@(t,y) linearized_pendulum_sys(t,y,x_rand), time_span, x_nearest, odeset('Events',@eventReachedThreshold));

    plot(y(:,1),y(:,2));
    hold on;
end

function [value,isterminal,direction] = eventReachedThreshold(t,y)
    threshold = 0.01;
    value      = double((y(1)*y(1) + y(2)*y(2)) < (threshold*threshold));
    isterminal = 1;
    direction  = 0;
end
