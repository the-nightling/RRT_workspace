function [P,path_handles] = rewire(V, P, X_near_indices, x_new, cost, n, path_handles)
% rewire tree such that vertices near x_new use x_new as parent if it is more cost-effective

    x_min_index = 1;
    delta_min = [];
    
    [Q,R] = get_LQR_cost_function_parameters();

    for x_near_index = X_near_indices'

        if(x_near_index ~= P(n))
            x_near = V(:,x_near_index);
            [t,delta,new_cost] = LQR_steer_connect(x_new,x_near);

            if(cost(n)+new_cost < cost(x_near_index))
                    P(x_near_index) = n;
                    delta_min = delta;
                    temp_handle = path_handles(x_near_index);
                    delete(temp_handle);
                    path_handles(x_near_index) = plot(delta_min(1:end,1),delta_min(1:end,2), 'Color', 'r');

            end
        end
    end
     
end
