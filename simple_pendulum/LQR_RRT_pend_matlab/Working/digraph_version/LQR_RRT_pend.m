function control = LQR_RRT_pend
% Computes the sequence of control actions needed for the swing up of
% a simple pendulum using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state (indicated by the red marker).
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.
	
	x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
	xG = [pi/2; 0];		% goal state

	xlimits = [-2*pi,pi; -10,10];	% state limits
		
	N = 1000;	% maximum number of iterations

	goal_bias = 0.10;   % choose the goal state instead of a random state 10% of the time

	V = digraph;        % RRT tree will be stored in a directed graph structure
	
	% initialise RRT root vertex properties
	cost = [0];         % stores cost of each node from root of tree (cummulative LQR cost of path through tree)
	path = {[0,0]};
	path_handle = [0];
	new_vertex = table(x0(1), x0(2), cost(1), path(1), path_handle(1), 'VariableNames', {'Position' 'Velocity' 'Cost' 'Path' 'Path_Handle'});
	V = addnode(V, new_vertex); % add root vertex to RRT tree
	
	setup_plot(x0,xG,xlimits);
	
	[t,delta_max,max_cost] = LQR_steer_connect(x0,xG);
	
	% keep growing RRT until goal found or run out of iterations
	for n = 2:N
	    use_goal = rand < goal_bias;
	    if(use_goal)
	        x_rand = xG;
	    else
            % get random state
        	x_rand = rand(2,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);
        end
		
		% select RRT vertex closest to the state point, based on LQR distance metric
		i = LQR_nearest([V.Nodes.Position,V.Nodes.Velocity]',x_rand);
		x_nearest = [V.Nodes.Position(i),V.Nodes.Velocity(i)]';
		
		% temporarily create branch from nearest tree vertex to the new random state
		[t, delta, new_cost] = LQR_steer_connect(x_nearest, x_rand);
        x_new = delta(end-1,:)';        % instead of the random state, use end of path steered towards random state; i.e. use x_new
        
        % get list of tree vertices near new state x_new
        X_near_indices = LQR_near([V.Nodes.Position,V.Nodes.Velocity]',x_new);
        
        % choose a parent for x_new such that adding x_new to the tree is most efficient in terms of cost
        [x_min_index, delta_min] = choose_parent([V.Nodes.Position,V.Nodes.Velocity]',X_near_indices,x_new,V.Nodes.Cost);
        if(length(delta_min) == 0)  % if no better parent found, choose nearest vertex as parent
            delta_min = delta;
            x_min_index = i;
        end
        x_new = delta_min(end-1,:)';
        
		% plot new RRT branch
        new_path_handle = plot(delta_min(1:end-1,1),delta_min(1:end-1,2), 'Color', 'b');
		
		% link new state to the nearest vertex in the tree
        new_vertex = table(x_new(1),x_new(2),V.Nodes.Cost(x_min_index)+new_cost,{delta_min(1:end-1,:)},new_path_handle,'VariableNames',{'Position' 'Velocity' 'Cost' 'Path' 'Path_Handle'});
        V = addnode(V, new_vertex);
        V = addedge(V, x_min_index, numnodes(V));

        % rewire tree such that vertices near x_new use x_new as parent is it is more cost-effective
        V = rewire(V, X_near_indices, x_new, numnodes(V));
        
        if(mod(n,10)==1)
            V = prune_tree(V,max_cost);
        end
        
		% for higher values of n, only update plot every 100 iteration (speeds up animation)
		%{
		if(mod(n,1)==1)
			drawnow;
		end
		%}
		
		drawnow;

		% if the goal was reached, stop growing tree
		%{
		if((x_new(1) <= xG(1)+0.1) && (x_new(1) >= xG(1)-0.1))
		    if((x_new(2) <= xG(2)+0.5) && (x_new(2) >= xG(2)-0.5))			
			    break;
			end
		end
		%}
		
	end
	
	if(n == N)
		title('Simulation complete (goal not found; ran out of iterations)');
	else		% retrace steps from goal state to initial state
	%{
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
    %}  
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
	plot(xG(1)-2*pi,xG(2),'r.','MarkerSize',30);	% goal state in red
	grid on;

	axis([xlimits(1,:),xlimits(2,:)]);
	xlabel('Angular position [rad]');
	ylabel('Angular velocity [rad/s]');
	
	set(gca,'XTick',-2*pi:pi/4:pi,'XTickLabel',{'-2pi','-7pi/4','-3pi/2','-5pi/4','-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});
end

