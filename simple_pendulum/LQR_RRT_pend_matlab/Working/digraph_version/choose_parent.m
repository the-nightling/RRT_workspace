function [x_min_index, delta_min] = choose_parent(V, X_near_indices, x_new, cost)
% choose a parent for x_new such that adding x_new to the tree is most efficient in terms of cost

    minCost = Inf;
    x_min_index = 1;
    delta_min = [];
    
    [Q,R] = get_LQR_cost_function_parameters();
    
    for x_near_index = X_near_indices'

        x_near = V(:,x_near_index);
        [t,delta,new_cost] = LQR_steer_connect(x_near,x_new);

        if(cost(x_near_index)+new_cost < minCost)
            minCost = cost(x_near_index)+new_cost;
            x_min_index = x_near_index;
            delta_min = delta;
        end
    end
    
end
