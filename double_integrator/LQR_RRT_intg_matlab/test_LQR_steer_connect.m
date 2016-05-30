function [] = test_LQR_steer_connect()
    x_nearest = [0;0;0;0];
    x_rand = [-1;0;-2;0];
    
    % call function
    [t,y,path_cost] = LQR_steer_connect(x_nearest,x_rand);

    % plot results for testing
    
	x0 = x_nearest;	% initial state; angle position measured from x-axis
	xG = x_rand;		% goal state

	figure(1);
	hold off;
	plot(x0(1),x0(3),'b.','MarkerSize',30);	% initial state in blue
	text(x_nearest(1),x_nearest(2),'  x_{nearest}');
	hold on;
	plot(xG(1),xG(3),'r.','MarkerSize',30);	% goal state in red
    text(x_rand(1),x_rand(3),'  x_{rand}');
	grid on;

	axis([-2,8,-8,8]);
	xlabel('x');
	ylabel('y');
	
    plot(y(:,1),y(:,3));
%    text(y(ceil(end/20),1),y(ceil(end/20),2),['path cost = ',num2str(path_cost)]);
    hold on;

end
