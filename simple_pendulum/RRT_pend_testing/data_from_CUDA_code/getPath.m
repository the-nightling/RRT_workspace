% Matlab script to build graph from
% adjacency matrix and find shortest path
% between two nodes

% copy graph data from GPU to local PC
pathToScript = fullfile(pwd,'copyGraphData.sh');
system([pathToScript]);

% import data into matlab workspace
adjMatrix = importdata('adjacency_matrix.txt');

% construct graph
graph = biograph(adjMatrix);
%view(graph);

% find Djikstra's shortest path
start_node = 976;	% 270
end_node = 998;		% 283
[dist,path,pred]=shortestpath(graph,start_node,end_node);
path
