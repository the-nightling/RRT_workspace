function [t,y,path_cost] = LQR_steer_connect(x_nearest,x_rand)
% returns a path y that links x_nearest to x_rand
% also returns the cost-to-go of the path

    time_step = get_time_step();
    time_span = [0:time_step:20];

    x_nearest_offsetted = x_nearest - x_rand;
    
    [A,B] = get_A_B_for_sys();
    [Q,R] = get_LQR_cost_function_parameters();
    
    % apply LQR control
    [K,S] = lqr(A,B,Q,R);

    [t,y_offsetted] = ode45(@(t,x) double_intg_sys(t,x,A,B,K), time_span, x_nearest_offsetted, odeset('Events',@eventReachedThreshold));

    y = y_offsetted + repmat(x_rand',length(y_offsetted),1);
    
    path_cost = [y(end-1,:)'- x_nearest]' * S * [y(end-1,:)'- x_nearest];

    %{
    % display control actions for path
    for i = 1:length(y)
        u = K * y_offsetted(i,:)'
    end
    %}
    
end

%%{
function [value,isterminal,direction] = eventReachedThreshold(t,y)
    threshold = 0.01;
    value      = double((y(1)*y(1) + y(2)*y(2)) < (threshold*threshold));
    isterminal = 1;
    direction  = 0;
end
%}
