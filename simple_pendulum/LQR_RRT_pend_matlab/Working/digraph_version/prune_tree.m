function V = prune_tree(V,max_cost)
    nodes_to_rm = [];
    
    
    
    for i = 1:numnodes(V)
        if(V.Nodes.Cost(i) > max_cost)
            nodes_to_rm = [nodes_to_rm,i];
        end
    end
    
    for node_index = nodes_to_rm
        delete(V.Nodes.Path_Handle);
    end
    
    V = rmnode(V,nodes_to_rm);
end
