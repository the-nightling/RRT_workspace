function control = LQR_RRT_pend
% Computes the sequence of control actions needed for the swing up of
% a simple pendulum using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state (indicated by the red marker).
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.
	
	x0 = [0; 0; 0; 0];	% initial state; angle position measured from x-axis
	xG = [8; 0; 0; 0];		% goal state

	xlimits = [-2,8; -8,8];	% state limits
		
	N = 1000;	% maximum number of iterations

	% pre-allocating memory
	V = repmat(x0,1,N);		% stores a graph of states in RRT
	P = ones(1,N);			% stores index of parent states
	Ui = ones(1,N);			% stores index of control actions in U
	u_path = ones(1,1000);  % stores sequence of control actions (solution to problem)
	xbi = 1;                % index used for traceback
	cost = [0];             % stores cost each node from root of tree (cummulative LQR cost of path through tree)
	paths = {[0,0]};
	path_handles = [0];
	
	setup_plot(x0,xG,xlimits);
	iteration = 2;
	
	% keep growing RRT util goal found or run out of iterations
	for n = 2:N
	
		% get random state
		x_rand = [0;0;0;0];
		x_rand(1) = rand(1,1)*10 - 2;
		x_rand(2) = rand(1,1)*20 - 10;
		x_rand(3) = rand(1,1)*16 - 8;
		x_rand(4) = rand(1,1)*20 - 10;
        
        %if(check_collision([x_rand]))
        %    continue;
        %end
        
%    	x_rand_handle = text(x_rand(1),x_rand(2),'  x_{rand}');
		
		% select RRT vertex closest to the state point, based on LQR distance metric
		i = LQR_nearest(V,x_rand,iteration);
		x_nearest = V(:,i);
%    	x_nearest_handle = text(x_nearest(1),x_nearest(2),'  x_{nearest}');
		
%		pause;
		
		% temporarily create branch from nearest tree vertex to the new random state
		[t, delta, new_cost] = LQR_steer_connect(x_nearest, x_rand);
        x_new = delta(end-1,:)';        % instead of the random state, use end of path steered towards random state; i.e. use x_new
        
        % get list of tree vertices near new state x_new
        X_near_indices = LQR_near(V,x_new,iteration);
        
        % choose a parent for x_new such that adding x_new to the tree is most efficient in terms of cost
        [x_min_index, delta_min] = choose_parent(V,X_near_indices,x_new,cost);
        if(length(delta_min) == 0)
            delta_min = delta;
            x_min_index = i;
        end
        x_new = delta_min(end-1,:)';
%        x_parent = V(:,x_min_index);
%    	x_parent_handle = text(x_parent(1),x_parent(2),'  x_{parent}');
        
%		pause;
        isColliding = check_collision(delta_min);

        if(~isColliding)
            
		    % plot new RRT branch
            new_path_handle = plot(delta_min(1:end-1,1),delta_min(1:end-1,3));
		
		    % link new state to the nearest vertex in the tree
    %        text(x_new(1),x_new(2),['',num2str(n)]);
		    V(:,iteration) = x_new;
		    P(iteration) = x_min_index;
		    cost = [cost; cost(x_min_index)+new_cost];
		    paths = {paths,delta_min(1:end-1,:)};
        	path_handles = [path_handles; new_path_handle];
    %		Ui(n) = ui;
            
            % rewire tree such that vertices near x_new use x_new as parent is it is more cost-effective
            [P,path_handles] = rewire(V, P, X_near_indices, x_new, cost, iteration, path_handles);
            iteration = iteration + 1
            
            drawnow;
		
%		    pause;
            
        end
		


		% if the goal was reached, stop growing tree
		%{
		if((x_new(1) <= xG(1)+0.1) && (x_new(1) >= xG(1)-0.1))
		    if((x_new(2) <= xG(2)+0.5) && (x_new(2) >= xG(2)-0.5))			
			    break;
			end
		end
		%}
		
	end
	
	if(iteration == N)
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
	plot(x0(1),x0(3),'b.','MarkerSize',30);	% initial state in blue
	hold on;
	plot(xG(1),xG(3),'r.','MarkerSize',30);	% goal state in red
	grid on;
	
	line([1,3],[-2,-2],'Color','k');
	line([1,1],[-2,8],'Color','k');
	line([3,3],[-2,8],'Color','k');
	line([1,3],[8,8],'Color','k');
	
	line([5,7],[-8,-8],'Color','k');
	line([5,5],[-8,2],'Color','k');
	line([7,7],[-8,2],'Color','k');
	line([5,7],[2,2],'Color','k');


	axis([xlimits(1,:),xlimits(2,:)]);
	xlabel('x');
	ylabel('y');
	
	set(gca,'XTick',-2:1:8);
end

