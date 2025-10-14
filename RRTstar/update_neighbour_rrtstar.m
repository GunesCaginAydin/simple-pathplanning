function [neighbour_update, min_costs, min_idxs] = update_neighbour_rrtstar(tree, change, configspc, neigh_parents)
    if change == 0
        neighbour_update = 0;
        min_costs = [];
        min_idxs = [];
        return

    end

    tree_len = length(tree);
    config_len = size(neigh_parents,1);
    neigh_update = [];
    min_costs = [];
    min_idxs = [];

    for crun = 1:config_len
        [~,neigh_update(crun), min_costs(crun), min_par_indx(crun)] = update_parent_rrtstar(tree, neigh_parents, configspc, crun);
    end

    function [parent_update, parent_change, cost_min, min_par_indx] = update_parent_rrtstar(tev, neigh_parent_list, fconfig_spc, crun) % possibility to write a collision check function
    parent_change = 0;

        tree_len = length(tev);
        config = neigh_parent_list(crun,:);   
        %parent_old = tev().parent;
    
        parent_list_no_col = col_check(config, neigh_parent_list, fconfig_spc);
        [cost_list_no_col, parent_idx_no_col] = tree_interp_cost(tev, parent_list_no_col);
        cost_add_no_col = dist(config, parent_list_no_col);
        cost_list_prov = cost_list_no_col + cost_add_no_col'; % FIX
    
        [cost_min,min_par_indx] = min(cost_list_prov,[],'all');
        
        %cost_list(min_par_indx) = cost_min;
        parent_update = parent_idx_no_col(min_par_indx);
    
        %if parent_update ~= parent_old
        %    parent_change = 1;
        %elseif parent_update == parent_old
        %    parent_change = 0;
        %    return
        %end

        function col_free_parents_coord = col_check(vertex,parent,config_space) % checks new parents for possible collisions
            lent = size(parent,1);
            col_free_parents_coord = [];
            for p = 1:lent
                space_ext_c = squeeze(abs(parent(p,2) - vertex(2)));
                space_ext_r = squeeze(abs(parent(p,1) - vertex(1)));
                space_ext_param = max([space_ext_c;space_ext_r]);
        
                configc_run = round(linspace(parent(p,2),vertex(2),space_ext_param));
                configr_run = round(linspace(parent(p,1),vertex(1),space_ext_param));
                configxy = [configr_run' configc_run'];
                line_idx = sub2ind(size(config_space), configxy(:,1), configxy(:,2));
    
                if any(config_space(line_idx) == 0)
                    continue
                elseif all(config_space(line_idx) == 1)
                    parent_coord = parent(p,:);
                    col_free_parents_coord = [col_free_parents_coord; parent_coord];
                else
                    disp('cannot resolve collisions, check free configuration space definition')
                end
            end
        end

        function [cost_list, idx] = tree_interp_cost(tree, col_free_pcoords)
            tparam = length(tree);
            pparam = size(col_free_pcoords,1);
            idx = [];
            cost_list = [];
            for j = 1:tparam
                for i = 1:pparam
                    if all(tree(j).vertex == col_free_pcoords(i,:))
                        idx = [idx; j];
                        cost_list = [cost_list; tree(j).totcost];
                    else
                        continue
                    end
                end
    
            end
            
        end

        function dist_add = dist(vertex, parent)
            lentn = size(parent,1);
            for k = 1:lentn
                vec = vertex - parent(k);
                dist_add(k) = norm(vec);
            end
        end
    end
end