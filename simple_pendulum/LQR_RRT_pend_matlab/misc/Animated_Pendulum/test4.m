clear;
tspan = [0 20];
iniCon = [-pi/2;0];
[t,y] = ode45(@pendulum_sys, tspan, iniCon);
control = [];

for i = 1:length(t)
    [yTemp,uTemp] = pendulum_sys(t(i),y(i,:)');
    control = [control;uTemp];
end

figure(1)
plot(t,y(:,1));
grid on;

figure(2)
plot(t,control);
grid on;

figure(3)
axis([-2,2,-2,2]);
grid on;
for n=1:length(y)
    ang_pos = y(n)+pi/2;
    XY_end_pos = [cos(ang_pos),sin(ang_pos)];
    
    cla(gca);
    hold on;
    plot(XY_end_pos(1),XY_end_pos(2),'b.','MarkerSize',50);    
    plot([0,XY_end_pos(1)],[0,XY_end_pos(2)],'LineWidth',3);
    hold off;
    drawnow;
end
