clear;
figure(1);
hold off;
path_handles = [0];
first_handle = plot(-pi/2,0,'b.','MarkerSize',30);	% initial state in blue
path_handles = [path_handles; first_handle];
hold on;

grid on;
xlimits = [-pi,pi; -10,10];	% state limits
axis([xlimits(1,:),xlimits(2,:)]);
xlabel('Angular position [rad]');
ylabel('Angular velocity [rad/s]');

set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});

pause;
path_handle = path_handles(2);
delete(path_handle);

pause;
path_handles(2) = plot(pi/2,0,'b.','MarkerSize',30);	% initial state in blue

pause;

path_handle = path_handles(2);
delete(path_handle);


