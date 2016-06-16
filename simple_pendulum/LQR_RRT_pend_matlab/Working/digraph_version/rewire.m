function V = rewire(V, X_near_indices, x_new, n)
% rewire tree such that vertices near x_new use x_new as parent if it is more cost-effective

    x_min_index = 1;
    delta_min = [];
    
    [Q,R] = get_LQR_cost_function_parameters();

    for x_near_index = X_near_indices'

        if(x_near_index ~= predecessors(V,n))
            x_near = [V.Nodes.Position(x_near_index),V.Nodes.Velocity(x_near_index)]';
            [t,delta,new_cost] = LQR_steer_connect(x_new,x_near);

            if(V.Nodes.Cost(n)+new_cost < V.Nodes.Cost(x_near_index))
                V = rmedge(V, predecessors(V,x_near_index), x_near_index);
                V = addedge(V, n, x_near_index);
                V.Nodes.Cost(x_near_index) = V.Nodes.Cost(n)+new_cost;

                delta_min = delta;
                temp_handle = V.Nodes.Path_Handle(x_near_index);
                delete(temp_handle);
                V.Nodes.Path_Handle(x_near_index) = plot(delta_min(1:end-1,1),delta_min(1:end-1,2), 'Color', 'r');

            end
        end
    end
     
end
