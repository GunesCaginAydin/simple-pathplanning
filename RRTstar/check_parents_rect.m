function [neighbour_parent_list,neighbour_parent_idx,neigh_found] = check_parents_rect(tev, r_search, c_search, fcs)
% RECT SEARCH - CIRCULAR SEARCH
    tree_len = length(tev);
    config = tev(tree_len).vertex;

    max_iter = 10;  neighbour_parent_list = []; neighbour_parent_idx = [];
    r_size = size(fcs,1); c_size = size(fcs,2);
    search_listr = config(1) - r_search/2 : config(1) + r_search/2;
    search_listr(search_listr<=1) = 1; search_listr(search_listr>=r_size) = r_size;
    search_listr = unique(search_listr);

    search_listc = config(2) - c_search/2 : config(2) + c_search/2;
    search_listc(search_listc<=1) = 1; search_listc(search_listc>=c_size) = c_size;
    search_listc = unique(search_listc);
    
    p_len = tree_len - 1;
    s = 1; size_it = 0;
    %r_search_new = r_search;
    %c_search_new = c_search;
    while s <= p_len
        if any(tev(s).vertex(1) == search_listr) && any(tev(s).vertex(2) == search_listc) && s <= p_len && size_it <= max_iter
            neighbour_parent = [tev(s).vertex(1) tev(s).vertex(2)];
            neighbour_parent_idx = [neighbour_parent_idx;s];
            neighbour_parent_list = [neighbour_parent_list;neighbour_parent];
        % elseif all(tev(s).vertex(1) ~= search_listr) || all(tev(s).vertex(2) ~= search_listc) && s <= p_len && size_it <= max_iter
        %     r_search_new = r_search_new + 10;
        %     c_search_new = c_search_new + 10;
        %     search_listr = config(1) - r_search_new/2 : config(1) + r_search_new/2;
        %     search_listr(search_listr<=1) = 1; search_listr(search_listr>=r_size) = r_size;
        %     search_listr = unique(search_listr);
        % 
        %     search_listc = config(2) - c_search_new/2 : config(2) + c_search_new/2;
        %     search_listc(search_listc<=1) = 1; search_listc(search_listc>=c_size) = c_size;
        %     search_listc = unique(search_listc);
        % 
        %     s = 1; size_it = size_it + 1;
        %     continue
        % elseif all(tev(s).vertex(1) ~= search_listr) || all(tev(s).vertex(2) ~= search_listc) && s <= p_len && size_it > max_iter
        %     neigh_found = 0;
        %     break
        % end
        end
        s = s + 1;
    end
    if isempty(neighbour_parent_list)
        neigh_found = 0;
    else
        neigh_found = 1;
    end
end