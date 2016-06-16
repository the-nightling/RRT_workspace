function nodes_to_rm = get_subtree_node_indices(V,root_index,nodes_to_rm)
    if(length(successors(V,root_index)) == 0)
        nodes_to_rm = [nodes_to_rm,root_index];
        return;
    else
        for node_index = [successors(V,root_index)]'
            nodes_to_rm = get_subtree_node_indices(V,node_index,nodes_to_rm);
        end
        nodes_to_rm = [nodes_to_rm,root_index];
    end
end
