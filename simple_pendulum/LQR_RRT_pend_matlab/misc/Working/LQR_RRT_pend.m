function control = LQR_RRT_pend
% Computes the sequence of control actions needed for the swing up of
% a simple pendulum using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state (indicated by the red marker).
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.
	
	x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
	xG = [pi/2; 0];		% goal state

	xlimits = [-pi,pi; -10,10];	% state limits
		
	N = 1000;	% maximum number of iterations

	% pre-allocating memory
	V = repmat(x0,1,N);		% stores a graph of states in RRT
	P = ones(1,N);			% stores index of parent states
	Ui = ones(1,N);			% stores index of control actions in U
	u_path = ones(1,1000);  % stores sequence of control actions (solution to problem)
	xbi = 1;                % index used for traceback
	cost = [0];             % stores cost each node from root of tree (cummulative LQR cost of path through tree)
	
	setup_plot(x0,xG,xlimits);
	
	% keep growing RRT util goal found or run out of iterations
	for n = 2:N
	
		% get random state
    	x_rand = rand(2,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);
		
		% select RRT vertex closest to the state point, based on LQR distance metric
		i = LQR_nearest(V,x_rand,n);
		x_nearest = V(:,i);
		
		% temporarily create branch from nearest tree vertex to the new random state
%		[t, delta, new_cost] = LQR_steer(x_nearest, x_rand);
		[t, delta, new_cost] = LQR_steer_connect(x_nearest, x_rand);
        x_new = delta(end-1,:)';        % instead of the random state, use end of path steered towards random state; i.e. use x_new
        
        % get list of tree vertices near new state x_new
        X_near_indices = LQR_near(V,x_new,n);
        
        % choose a parent for x_new such that adding x_new to the tree is most efficient in terms of cost
        [x_min_index, delta_min] = choose_parent(V,X_near_indices,x_new,cost);
        x_new = delta(end-1,:)';
        		
		% if angular position is greater than pi rads, wrap around
		temp = x_new(1);
		if( (x_new(1) > pi) || (x_new(1) < -pi) )
        		x_new(1) = mod(x_new(1)+pi,2*pi)-pi;
        end
		
		% plot new RRT branch
		if(abs(x_new(1)-temp) < pi)
            plot(delta(1:end-1,1),delta(1:end-1,2));
		%{
			line([V(1,i),x_new(1)],[V(2,i),x_new(2)],'Color','b');
		%}
		end

		
		% link new state to the nearest vertex in the tree
		V(:,n) = x_new;
		P(n) = x_min_index;
		cost = [cost; cost(x_min_index)+new_cost];
%		Ui(n) = ui;
        
        % rewire tree such that vertices near x_new use x_new as parent is it is more cost-effective
        P = rewire(V, P, X_near_indices, x_new, cost, n);
        
		% for higher values of n, only update plot every 100 iteration (speeds up animation)
		%{
		if(mod(n,1)==1)
			drawnow;
		end
		%}
		
		drawnow;
%		pause;

		% if the goal was reached, stop growing tree
		if((x_new(1) <= xG(1)+0.1) && (x_new(1) >= xG(1)-0.1))
		    if((x_new(2) <= xG(2)+0.5) && (x_new(2) >= xG(2)-0.5))			
			    break;
			end
		end
		
	end
	
	if(n == N)
		title('Simulation complete (goal not found; ran out of iterations)');
	else		% retrace steps from goal state to initial state
	    title('Simulation complete (goal found)');
	    n
        xbi = n;
        index = 1;

        % retrace control actions and solution trajectory
		% path displayed using red line
        while(xbi ~= 1)
	        xx = [V(1,xbi),V(1,P(xbi))];
	        if abs(xx(2)-xx(1))<pi
		        line([V(1,xbi),V(1,P(xbi))],[V(2,xbi),V(2,P(xbi))],'Color','r','LineWidth',2);
	        end
	
%            u_path(index) = U( Ui(xbi) );
%            index = index+1;
	        xbi = P(xbi);
        end
        
%{        
        index2 = 1;
       	u_path_final = ones(1,index-1);
        while(index > 1)
            u_path_final(index2) = u_path(index-1);
            index = index-1;
            index2 = index2+1;
        end
        
        control = [[0:dt:dt*length(u_path_final)-dt]',u_path_final'];
%}
        drawnow;

	end
	
end

function [] = setup_plot(x0,xG,xlimits)
    % setup plot
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
end

