function [t,y,path_cost] = LQR_steer(x_nearest,x_rand)
% returns a path y that attempts to link x_nearest to x_rand
% the path is truncated after a set number of time steps
% also returns the cost-to-go of the path

    time_step = get_time_step();
    time_span = [0:time_step:100*time_step];

    x_nearest_offsetted = x_nearest - x_rand;
    
    % linearize non-linear pendulum about x_rand
    [A,B] = linearize_pendulum_about(x_rand);
    [Q,R] = get_LQR_cost_function_parameters();
    
    % apply LQR control
    [K,S] = lqr(A,B,Q,R);
    
    [t,y_offsetted] = ode45(@(t,y) linearized_pendulum_sys(t,y,x_rand,A,B,K), time_span, x_nearest_offsetted);
%    [t,y_offsetted] = ode45(@(t,y) linearized_pendulum_sys(t,y,x_rand,A,B,K), time_span, x_nearest_offsetted, odeset('Events',@eventReachedThreshold));

    y = y_offsetted + repmat(x_rand',length(y_offsetted),1);
    
    path_cost = [y(end,:)'- x_nearest]' * S * [y(end,:)'- x_nearest];
    
%{
    % plot results for testing
    
	x0 = x_nearest;	% initial state; angle position measured from x-axis
	xG = x_rand;		% goal state
	xlimits = [-pi,pi; -10,10];	% state limits
	figure(1);
	hold off;
	plot(x0(1),x0(2),'b.','MarkerSize',30);	% initial state in blue
	hold on;
	plot(xG(1),xG(2),'r.','MarkerSize',30);	% goal state in red
	grid on;

	axis([xlimits(1,:),xlimits(2,:)]);
	xlabel('Angular position [rad]');
	ylabel('Angular velocity [rad/s]');
	
	set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});
    plot(y(:,1),y(:,2));
    hold on;
%}
end

%{
function [value,isterminal,direction] = eventReachedThreshold(t,y)
    threshold = 0.01;
    value      = double((y(1)*y(1) + y(2)*y(2)) < (threshold*threshold));
    isterminal = 1;
    direction  = 0;
end
%}
