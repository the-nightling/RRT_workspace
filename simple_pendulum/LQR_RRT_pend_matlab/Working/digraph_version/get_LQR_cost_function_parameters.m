function [Q,R] = set_LQR_cost_function_parameters()
    Q = diag([1 1]);    % penalizes states
    R = 0.05;            % penalizes control actions
end
