clear
clc
close

%%
map_rgb = imread('map_4_d.png'); % read the image
map_rgb_resized = imresize(map_rgb,[1000 1000]); % scale the image
[fcs, oc, os, cs] = create_config_space(map_rgb_resized);

[gr_row, gr_col, r_row, r_col, ~, ~] ...
    = color_coordinates(map_rgb_resized); % identify regions of colors

x_scale = [0 size(map_rgb_resized,1)]; % scale the problem in x = columns
y_scale = [0 size(map_rgb_resized,2)]; % scale the problem in y = rows

start_pos = [gr_row gr_col]; % define start position in (x,y) config
end_pos = [r_row r_col]; % define goal position in (x,y) config

xlim(x_scale); ylim(y_scale)
plot(start_pos(2),start_pos(1),'Marker','o','MarkerFaceColor','g')
plot(end_pos(2),end_pos(1),'Marker','o','MarkerFaceColor','r')
clear map_rgb_resized map_rgb

%%
il = 1e5;
it = 0;
tev = struct('vertex',{},'initial',{},'parent',{},'col',{},'mindist',{},'totcost',{},'line',{}); % collection of tree evolution - costp cost to parent
tev(1).vertex = start_pos; % initial configuration from start position
tev(1).initial = 0; % initial configuration
tev(1).parent = 0; % parent of each configuration
tev(1).mindist = 0; %min distance to parent node - obtained with other functions
tev(1).totcost = 0; %total cost of node - obtained with min distances
tev(1).col = 0; % first collision check RRT
tev(1).line = 0; % line object to be modified

r_search = 150; % RRT* search param1 - width
c_search = 150; % RRT* search param2 - height
rc_search = 150; % RRT* search param3 for circular search radius
cost_lim = Inf; % initial cost argument, to be updated later through user
comp_scale = 30/1000; % scale for comparison with searchbased algorithms
dist_bias = 0; % random distribution bias around the goal position, [0-1]

%%
tic
while it < il
    it = it + 1;

    rp = [randi(x_scale(2:end)) randi(y_scale(2:end))]; % choose random coordinates
    tev(it+1).vertex = rp; % initial vertex
    tev(it+1).initial = rp;
    [~, ~, ~, ~, I] = calc_edges(tev);
    tev(it+1).parent = I; % initial parent

    [rp, col_check] = check_collision_and_project(tev(I).col,tev(I).vertex,rp,fcs); % for exploration of obstacles
    if isnan(rp) % rp doesnt fit above criteria, reassign random rp & turn back one step
        it = it - 1; 
        continue
    end
    tev(it+1).vertex = rp; % final vertex 
    tev(it+1).col = col_check; % collision check
    [~, ~, dist_min, ~, ~] = calc_edges(tev);
    tev(it+1).mindist = dist_min;

    [neighbour_parent_list, neighbour_parent_idx,found] = check_parents_rect(tev, r_search, c_search, fcs); % check possible parents around a search area RRT* |||| all verteces
    %[neighbour_parent_list, neighbour_parent_idx,found] = check_parents_circ(tev, rc_search, fcs);
    if found == 0
        it = it - 1;
        continue
    end

    tev = calc_costs_from_distances(tev); % calculate costs from distances to parents field - first trial - to be updated after new 

    [parent_update, parent_change, minp_cost, minp_dist, minp_idx, col, neighbour_parent_idx_no_col, neighbour_parent_list_no_col] = update_parent_rrtstar(tev, neighbour_parent_list, neighbour_parent_idx, fcs); % update parent according to cost RRT*
    if col == 1
        it = it - 1;
        continue
    end
    tev(it+1).parent = parent_update; % parent update
    tev(it+1).mindist = minp_dist; % distance update
    tev(it+1).totcost = minp_cost; % cost update

    goal_dist = cal_dist(end_pos,tev(it+1).vertex);

    plot(tev(it+1).vertex(2),tev(it+1).vertex(1),'o','Color','b','LineWidth',1)
    line_to_par = line([tev(parent_update).vertex(2),tev(it+1).vertex(2)],...
                [tev(parent_update).vertex(1),tev(it+1).vertex(1)],'LineWidth',1,'Color','b');
    tev(it+1).line = line_to_par;
    drawnow

    if parent_change == 1
        [tev, update] = update_vicinity_rrtstar(tev, neighbour_parent_idx_no_col, neighbour_parent_list_no_col, it+1); % WORK
    end

    tev = calc_costs_from_distances(tev); 
    
    if goal_dist <= 50 && tev(it+1).totcost <= cost_lim
        current_config = it + 1;
        path_dist = 0;
            
        while current_config ~= 1
            parent_config = tev(current_config).parent;
            lc{current_config} = line([tev(parent_config).vertex(2),tev(current_config).vertex(2)],...
                [tev(parent_config).vertex(1),tev(current_config).vertex(1)],...
                'Color','r','LineWidth',2);
            pc{current_config} = plot(tev(parent_config).vertex(2),tev(parent_config).vertex(1),...
                'Marker','diamond','MarkerFaceColor','c','MarkerSize',7.5);
            drawnow
            path_dist = path_dist + tev(current_config).mindist;
            current_config = parent_config;
        end 


        memory_cons = memory().MemUsedMATLAB/(1024^3);
        str = sprintf(['\n --- Next Run --- \n\nGoal reached within a distance ' ...
            'of %f units\npath distance: %f units\niterations required: %d\nMemory ' ...
            'Consumption: %f GB\n\n'],goal_dist*comp_scale, path_dist*comp_scale, ...
            it, memory_cons);
        disp(str);
        toc
    else
        continue
    end

    inp_prompt = {'Continue search for refinement of grid - reduction of total cost?','Percent cost reduction:'};
    dlgtitle = 'Search Interrupt';
    fieldsize = [1 100; 1 100];
    definput = {'-','-'};
    cost_arg = inputdlg(inp_prompt,dlgtitle,fieldsize,definput);
    if isempty(cost_arg)
        break
    elseif cost_arg{1} == '1'
        cost_lim = tev(end).totcost - str2double(cost_arg{2})/100*tev(end).totcost;
        for a = 1:length(lc)
            delete(lc{a})
            delete(pc{a})
        end
        continue
    elseif cost_arg{1} == '0'
        break
    end   
end

if it == il
    disp('iteration cap reached, no further improvement is possible')
end
