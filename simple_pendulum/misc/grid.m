clear;
clf;

x_min = -2;
x_max = 3;
y_min = -4;
y_max = 5;
z_min = -6;
z_max = 7;

x_nodes = 2;
y_nodes = 3;
z_nodes = 4;

xlimits = [x_min,x_max; y_min,y_max; z_min,z_max];	% state limits
%xlimits = [x_min,x_max; y_min,y_max];	% state limits

delta_x = (x_max-x_min)/(2*x_nodes+2);
delta_y = (y_max-y_min)/(2*y_nodes+2);
delta_z = (z_max-z_min)/(2*z_nodes+2);

figure(1);
hold on;
axis([x_min,x_max,y_min,y_max,z_min,z_max]);
%axis([x_min,x_max,y_min,y_max]);

for idx = 0:(x_nodes*y_nodes*z_nodes-1)
    x_coord = x_min + ( mod(idx,x_nodes)*2*delta_x ) + 2*delta_x;
    y_coord = y_min + ( mod(floor(idx/x_nodes),y_nodes)*2*delta_y ) + 2*delta_y;
    z_coord = z_min + ( mod(floor(idx/(x_nodes*y_nodes)),z_nodes)*2*delta_z ) + 2*delta_z;

    plot3(x_coord,y_coord,z_coord,'.','MarkerSize',30);
%    plot(x_coord,y_coord,'.','MarkerSize',30);
    

    for goal_idx = 0:(3^3)-1
        goal_x = x_coord + (mod(goal_idx,3)-1)*2*delta_x;
        goal_y = y_coord + (mod(floor(goal_idx/3),3)-1)*2*delta_y;
        goal_z = z_coord + (mod(floor(goal_idx/(3*3)),3)-1)*2*delta_z;
        plot3(goal_x,goal_y,goal_z,'r.','MarkerSize',10);
%        plot(goal_x,goal_y,'r.','MarkerSize',10);
        
%        lim_x = x_coord + (mod(goal_idx,3)-1)*3*delta_x;
%        lim_y = y_coord + (mod(floor(goal_idx/3),3)-1)*3*delta_y;
%        lim_z = z_coord + (mod(floor(goal_idx/(3*3)),3)-1)*3*delta_z;
%        plot3(lim_x,lim_y,lim_z,'k.','MarkerSize',10);
    end

    
end
