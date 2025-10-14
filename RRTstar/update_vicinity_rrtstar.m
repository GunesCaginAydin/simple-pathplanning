function [tree_new, update] = update_vicinity_rrtstar(tree, neigh_idx, neigh, iter)
    tree_new = tree;

    config = tree_new(end).vertex;
    neigh_len = length(neigh_idx);

    for narg = 1:neigh_len
        neigh_costs_past(narg) = tree_new(neigh_idx(narg)).totcost;
    end
    add_dists = dist(config,neigh);
    neigh_costs_new = tree_new(end).totcost + add_dists;

    if any(neigh_costs_new < neigh_costs_past)

        I = neigh_costs_new < neigh_costs_past;
        %nnew = neigh_costs_new(I);
        %nold = neigh_costs_past(I);
        change_idx = neigh_idx(I);
        dists = add_dists(I);
        crun = length(change_idx);

        for carg = 1:crun
            currentn = change_idx(carg);
            
            delete(tree_new(currentn).line)
            %new_line = line([tree_new(currentn).vertex(2),config(2)],...
            %    [tree_new(currentn).vertex(1),config(1)],'LineWidth',1,'Color','b');
            new_line = line([config(2),tree_new(currentn).vertex(2)],...
                [config(1),tree_new(currentn).vertex(1)],'LineWidth',1,'Color','b');            
            tree_new(currentn).line = new_line;

            tree_new(currentn).mindist = dists(carg);
            tree_new(currentn).parent = iter;
            %tree(currentn).totcost = neigh_costs_new(I);
            %[childs, chilnum] = find_children(tree, change_idx(crun));
            %cost_change = nold(carg) - nnew(carg); 

            % for chilarg = 1:chilnum
            %     tree(childs(chilarg)).totcost = tree(childs(chilarg)).totcost - cost_change;                
            % end

        end

        update = 1;

    elseif all(neigh_costs_new >= neigh_costs_past)

        update = 0;
        return

    end

    % function [child_idx child_num] = find_children(tree, inpid)
    % 
    % 
    %     return   
    % end

    function dist_add = dist(vertex, parent)
        lentn = size(parent,1);
        for k = 1:lentn
            vec = vertex - parent(k);
            dist_add(k) = norm(vec);
        end
    end
end