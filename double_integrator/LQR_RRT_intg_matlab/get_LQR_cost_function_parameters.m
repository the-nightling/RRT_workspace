function [Q,R] = get_LQR_cost_function_parameters()
    Q = diag([0.01 0.01 0.01 0.01]);    % penalizes states
%    Q = zeros(4,4);
    R = diag([1 1]);            % penalizes control actions
end
