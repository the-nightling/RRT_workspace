function X_near_indices = LQR_near(V, x, n, Q, R)

    [A,B] = linearize_pendulum_about(x);

    [K,S] = lqr(A,B,Q,R);
    
    X_near_indices = [];
    constant_gamma = 2;
    threshold = constant_gamma * sqrt(log(n)/n);
    
    for i = 1:n-1
        cost_to_go = [V(:,i)-x]' * S * [V(:,i)-x];
        if(cost_to_go <= threshold)
            X_near_indices = [X_near_indices; i];
        end
    end
    
end