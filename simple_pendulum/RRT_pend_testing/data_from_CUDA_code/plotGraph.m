% code to draw graph

% copy tree data from GPU to local PC
pathToScript = fullfile(pwd,'copyTreeData.sh');
system([pathToScript]);

T = importdata('path_solutions.txt');

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

axis([-pi,pi,-10,10]);

xlabel('Angular position [rad]');
ylabel('Angular velocity [rad/s]');

grid on;
set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});

for j=1:length(roots)
%    line([roots(1,j),goals(1,j)],[roots(2,j),goals(2,j)],'Color','b');
    quiver(roots(1,j),roots(2,j),goals(1,j)-roots(1,j),goals(2,j)-roots(2,j),0, 'MaxHeadSize', 0.8);
end
