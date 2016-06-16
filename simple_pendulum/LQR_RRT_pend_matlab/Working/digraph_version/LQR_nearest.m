function x_nearest_index = LQR_Nearest(V,x)
% return the index of the tree vertex that is closest to a particular point x,
% using an LQR-based distance metric

    [A,B] = linearize_pendulum_about(x);
    [Q,R] = get_LQR_cost_function_parameters();
    
    [K,S] = lqr(A,B,Q,R);
    
    x_nearest_index = 1;
    min_cost_to_go = 999;
        
    for i = 1:length(V(1,:))
        new_cost_to_go = [V(:,i)-x]' * S * [V(:,i)-x];
        if(new_cost_to_go < min_cost_to_go)
            min_cost_to_go = new_cost_to_go;
            x_nearest_index = i;
        end
    end
    
end
