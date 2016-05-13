function x_nearest_index = LQR_Nearest(V,x,n)

    [A,B] = linearize_pendulum_about(x);
    Q = diag([1 10]);
    R = 1;
    [K,S] = lqr(A,B,Q,R);
    
    x_nearest_index = 1;
    min_cost_to_go = 999;
    
    for i = 1:n
        new_cost_to_go = [V(:,i)-x]' * S * [V(:,i)-x];
        if(new_cost_to_go < min_cost_to_go)
            min_cost_to_go = new_cost_to_go;
            x_nearest_index = i;
        end
    end
    
end
