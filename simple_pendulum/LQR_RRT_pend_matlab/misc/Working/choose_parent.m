function [x_min_index, delta_min] = choose_parent(V, X_near_indices, x_new, cost, Q, R)
    minCost = Inf;
    x_min_index = 1;
    delta_min = [];
    
    for x_near_index = X_near_indices'

        x_near = V(:,x_near_index);
        [t,delta] = LQR_steer(x_near,x_new, Q, R);
        new_cost = t(end-1);
        
        if(cost(x_near_index)+new_cost < minCost)
            minCost = cost(x_near_index)+new_cost;
            x_min_index = x_near_index;
            delta_min = delta;
        end
    end
end
