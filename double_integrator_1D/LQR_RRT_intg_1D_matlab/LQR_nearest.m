function x_nearest_index = LQR_Nearest(V,x,n)
% return the index of the tree vertex that is closest to a particular point x,
% using an LQR-based distance metric

    [A,B] = get_A_B_for_sys();
    [Q,R] = get_LQR_cost_function_parameters();
    
    [K,S] = lqr(A,B,Q,R);
    
    x_nearest_index = 1;
    min_cost_to_go = inf;
    
    for i = 1:n-1
        new_cost_to_go = [V(:,i)-x]' * S * [V(:,i)-x];
        if(new_cost_to_go < min_cost_to_go)
            min_cost_to_go = new_cost_to_go;
            x_nearest_index = i;
        end
    end
    
end