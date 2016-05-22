function [] = test_LQR_steer()
    x_nearest = [-pi/2;0];
    x_rand = [-pi/4;4];
    
    % call function
    [t,y,path_cost] = LQR_steer(x_nearest,x_rand);
    
    [t_past_limit,y_past_limit,path_cost_past_limit] = LQR_steer_connect(x_nearest,y(end,:)');
    path_cost_past_limit
    
    % plot results for testing
    
	x0 = x_nearest;	% initial state; angle position measured from x-axis
	xG = x_rand;		% goal state
	xlimits = [-pi,pi; -10,10];	% state limits
	figure(1);
	hold off;
	plot(x0(1),x0(2),'b.','MarkerSize',30);	% initial state in blue
	text(x_nearest(1),x_nearest(2),'  x_{nearest}');
	hold on;
	plot(xG(1),xG(2),'r.','MarkerSize',30);	% goal state in red
    text(x_rand(1),x_rand(2),'  x_{rand}');
	grid on;

	axis([xlimits(1,:),xlimits(2,:)]);
	xlabel('Angular position [rad]');
	ylabel('Angular velocity [rad/s]');
	
	set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});
    plot(y(:,1),y(:,2));
    text(y(end,1),y(end,2),['  path cost = ',num2str(path_cost)]);
    plot(y(end,1),y(end,2),'bx');
    plot(y_past_limit(:,1),y_past_limit(:,2),'r');
    hold on;
    
end
