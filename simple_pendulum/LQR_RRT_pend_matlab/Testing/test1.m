clf;
clear;
figure(1);
axis([-2,2,-2,2]);
grid on;

initial_ang_pos = pi/2;
final_ang_pos = 2*pi/3;
ang_pos = pi/2;

hold on;
XY_end_pos = [cos(ang_pos),sin(ang_pos)];
plot(XY_end_pos(1),XY_end_pos(2),'b.','MarkerSize',50);
plot([0,XY_end_pos(1)],[0,XY_end_pos(2)],'LineWidth',3);
hold off;
drawnow;

for n=1:100
    ang_pos = ang_pos+(final_ang_pos-initial_ang_pos)/100;
    XY_end_pos = [cos(ang_pos),sin(ang_pos)];
    
    cla(gca);
    hold on;
    plot(XY_end_pos(1),XY_end_pos(2),'b.','MarkerSize',50);    
    plot([0,XY_end_pos(1)],[0,XY_end_pos(2)],'LineWidth',3);
    hold off;
    drawnow;
end
