function P = rewire(V, P, X_near_indices, x_new, cost, n)
    x_min_index = 1;
    delta_min = [];
    
    [Q,R] = set_LQR_cost_function_parameters();

    for x_near_index = X_near_indices'

        x_near = V(:,x_near_index);
        [t,delta] = LQR_steer(x_new,x_near);
        new_cost = t(end-1);
        
        if(cost(n)+new_cost < cost(x_near_index))
            P(x_near_index) = n;
            delta_min = delta;
            plot(delta_min(1:end-1,1),delta_min(1:end-1,2), 'Color', 'r');
        end
    end
     
end
