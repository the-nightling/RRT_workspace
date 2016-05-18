function [Q,R] = set_LQR_cost_function_parameters()
    Q = diag([1 1]);
    R = 1;
end
