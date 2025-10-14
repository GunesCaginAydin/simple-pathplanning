clear
clc
close

%% SYSTEM DEFINITIONS - IMAGE ANALYSIS, PARAMETRIZATION AND PROBLEM CLASSES
%[~, map_folder] = add_path_to_scripts; % add script folder and map folder
[~, map] = add_path_to_scripts_and_map;
resize_to = [1000 1000]; % resize argument
%[~, map_rgb_resized, ~] = read_and_resize_map(map_folder, resize_to);
map_rgb_resized = read_and_resize_map_selection(map,resize_to);
%map_rgb_resized = map_rgb_resized{1};
[fcs, oc, os, cs] = create_config_space(map_rgb_resized);
[gr_row, gr_col, r_row, r_col, ~, ~] = color_coordinates(map_rgb_resized); 
% identify regions of colors

x_scale = [0 size(map_rgb_resized,1)]; % scale the problem in x = columns
y_scale = [0 size(map_rgb_resized,2)]; % scale the problem in y = rows

start_pos = [gr_row gr_col]; % define start position in (x,y) config
end_pos = [r_row r_col]; % define goal position in (x,y) config
xlim(x_scale); ylim(y_scale)
plot(start_pos(2),start_pos(1),'Marker','o','MarkerFaceColor','g')
plot(end_pos(2),end_pos(1),'Marker','o','MarkerFaceColor','r')
clear map_rgb_resized % clear variables to not overcrowd the space

%% VARIABLE INITIALIZATION FOR RRT
% Tree struct is utilized in obtaining the necessary parameteres at each
% iteration, vertex, parent data, collision check, initial vertex selection
% and edge parameters such as slope and length are stored in the necessary
% allocated fields, throughout the tree evolution the parameters are
% constantly refreshed according to min distance laws of RRT
il = 1e5; % iteration cap
it = 0; % iteration init

tev = struct('vertex',{},'initial',{},'col',{},'edge',{},'parent',{}); 
% collection of tree evolution
tev(1).vertex = start_pos; % initial vertex = start vertex
tev(1).parent = 0; % parent init
tev(1).edge = 0; % edge data init
tev(1).col = 0; % collision data init
tev(1).initial = 0; % initial vertex data init

path_dist = 0; % path distance init
comp_scale = 30/1000;

dist_bias = 0; % random distribution bias around the goal position, [0-1]

%% SYSTEM INITIALIZATION FOR PATH FINDING ALGORITHM - RRT
% RRT trees are obtained through random trees sampled in the search space.
% Each sample is attempted through a connection between the start point and
% the closest points to the sample, for a feasible connection the sample is
% admitted. Growth factor (weight) and probabilistic goal sampling (bias)
% to increease the growth rate, random samples to asses growth direction, q
% relates to the position of the point on configuration space, extensions
% in considerration of the orientation are also possible 
tic
while it < il % while iteration cap is not reached
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
    [dists, angles, dist_min, angle_min, ~] = calc_edges(tev); % calc edge values
    tev(it+1).edge = [dist_min angle_min]; % edge dist/angle polarity
    
    plot(tev(it+1).vertex(2),tev(it+1).vertex(1),'o','Color','b','LineWidth',1)
    line([tev(I).vertex(2),tev(it+1).vertex(2)],...
        [tev(I).vertex(1),tev(it+1).vertex(1)],'LineWidth',1,'Color','b')
    drawnow

    goal_dist = cal_dist(end_pos,tev(it+1).vertex);

    if goal_dist <= 35
        current_config = it + 1;
        while current_config ~= 1
            parent_config = tev(current_config).parent;
            line([tev(parent_config).vertex(2),tev(current_config).vertex(2)],...
                [tev(parent_config).vertex(1),tev(current_config).vertex(1)],...
                'Color','r','LineWidth',4)
            plot(tev(parent_config).vertex(2),tev(parent_config).vertex(1),...
                'Marker','diamond','MarkerFaceColor','c','MarkerSize',7.5)
            drawnow
            path_dist = path_dist + tev(current_config).edge(1);
            current_config = parent_config;
        end
        break
    end

end

if it == il
    fig = uifigure;
    uialert(fig,['Iteration cap reached, no convergence: no connection' ...
        ' present'], 'Exceed Iteration')
end

%% CLASSIFICATION AND COMPARISON
toc
memory_cons = memory().MemUsedMATLAB/(1024^3);
str = sprintf(['Goal reached within a distance of %f units\npath' ...
    ' distance: %f units\niterations required: %d\nMemory ' ...
    'Consumption: %f GB'],goal_dist*comp_scale, path_dist*comp_scale, ...
    it, memory_cons);
disp(str);