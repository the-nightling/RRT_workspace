V = digraph;
%RRT_tree = addnode(RRT_tree, 1);
%RRT_tree.Nodes.Position = -pi/2;
%RRT_tree.Nodes.Velocity = 0.0;
x0 = [1.1; 0];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [2.2; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [3.3; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [4.4; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [5.5; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [6.6; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [7.7; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [7.7; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [7.7; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);

x0 = [7.7; 0.1];
new_vertex = table(x0(1),x0(2),'VariableNames',{'Position' 'Velocity'});
V = addnode(V, new_vertex);



V = addedge(V,1,2);
V = addedge(V,1,3);
V = addedge(V,1,4);
V = addedge(V,2,5);
V = addedge(V,2,6);
V = addedge(V,3,7);
V = addedge(V,6,8);
V = addedge(V,6,9);
V = addedge(V,6,10);

plot(V);

