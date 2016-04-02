% code to draw usable paths (blue) and shortest path (red)

% copy tree data from GPU to local PC
pathToScript = fullfile(pwd,'copyTreeData.sh');
system([pathToScript]);

T = importdata('path_solutions.txt');

getPath;

j = 1;
goals = ones(2,16128);
roots = ones(2,16128);
for i=40:40:length(T)
    goals(1,j) = T(i-1);
    goals(2,j) = T(i);
    roots(1,j) = T(i-3);
    roots(2,j) = T(i-2);
    j = j+1;
end

figure(2)
x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
xG = [pi/2; 0];		% goal state

hold off
plot(x0(1),x0(2),'g.','MarkerSize',30);	% initial state in blue
hold on
plot(xG(1),xG(2),'r.','MarkerSize',30);	% goal state in red

axis([-pi,pi,-10,10])

xlabel('Angular position [rad]');
ylabel('Angular velocity [rad/s]');

grid on;
set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});

%X = ones(2,230400);
for i=1:2:length(T)
%    X(1,fix(i/2)+1) = T(i);
%    X(2,fix(i/2)+1) = T(i+1);
    if(~(T(i)==0 && T(i+1)==0))
        plot(T(i),T(i+1))
    end
end



for j=1:length(path)
    for i=1:8
        if(~(roots(1,path(j)*8-8+i) ==0 && roots(2,path(j)*8-8+i)==0))
            plot(roots(1,path(j)*8-8+i),roots(2,path(j)*8-8+i),'r.','MarkerSize',15);
        end
    end
end
