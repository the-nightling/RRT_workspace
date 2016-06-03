goal_bias = 0.05;
xG = [8; 0; 0; 0];		% goal state
n = 2;
while n < 100
    try_goal = rand < goal_bias
    if try_goal
        x_rand = xG;
    else
   		x_rand = [0;0;0;0];
		x_rand(1) = rand(1,1)*10 - 2;
		x_rand(2) = rand(1,1)*20 - 10;
		x_rand(3) = rand(1,1)*16 - 8;
		x_rand(4) = rand(1,1)*20 - 10;
        %check_constraints
    end
    x_rand
    n = n+1;
    pause
end
