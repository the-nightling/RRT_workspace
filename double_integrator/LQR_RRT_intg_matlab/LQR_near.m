function X_near_indices = LQR_near(V, x, n)
% return the a list of indices of tree vertices that are within a certain distance from particular point x,
% using an LQR-based distance metric

    [A,B] = get_A_B_for_sys();
    [Q,R] = get_LQR_cost_function_parameters();
    
    [K,S] = lqr(A,B,Q,R);
    
    X_near_indices = [];
    constant_gamma = 100;
    threshold = constant_gamma * sqrt(log(n)/n);
    
    for i = 1:n-1
        cost_to_go = [V(:,i)-x]' * S * [V(:,i)-x];
        if(cost_to_go <= threshold)
            X_near_indices = [X_near_indices; i];
        end
    end
    
end
