function [dists, angles, dist_min, angle_min, indx_min] = calc_edges(tree)
% calculate the distances betweeen first proposed random point [vertex] and
% each of the possible other verteces [parent] in the tree
    %treecell = struct2cell(tree.vertex);
    tree_len = length(tree);
    parents = zeros(tree_len-1,2); vecs = parents;
    dists = zeros(tree_len-1,1); angles = dists;
    vertex = tree(tree_len).vertex;
    for tree_arg = 1:(tree_len-1)
        parents(tree_arg,:) = tree(tree_arg).vertex;
        vecs(tree_arg,:) = vertex - parents(tree_arg,:);
        dists(tree_arg) = norm(vecs(tree_arg,:));
        %angles(tree_arg) = angle(vecs(tree_arg,:));
    end


    
    [dist_min, indx_min]  = min(dists,[],'all');
    %angle_min = angles(indx_min);
    angles = 0;
    angle_min = 0;
end