clear
clc
close
%[~,maps] = add_path_to_scripts;
%[map_rgb, map_rgb_resized] = read_and_resize_map(maps);
map_rgb = imread(map_rgb); % read the image
map_rgb_resized = imresize(map_rgb,[1000 1000]); % scale the image
[fcs, oc, os, cs] = creat_config_space(map_rgb_resized);
%imshow(map_rgb_resized) % show image to work on
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


il = 1e5;
it = 0;
tev = struct('vertex',{},'initial',{},'parent',{},'col',{},'edge',{}); % collection of tree evolution
tev(1).vertex = start_pos; tev(1).parent = 0; tev(1).edge = 0; tev(1).col = 0; tev(1).initial = 0;
while it < il
    it = it + 1;

    rp = [randi(x_scale(2:end)) randi(y_scale(2:end))]; % choose random coordinates
    [dists, angles, dist_min, angle_min, indx_min] = calc_edges(tev); % calc edge values
    tev(it+1).initial = rp; % initial vertex
    tev(it+1).edge = [dist_min angle_min]; % edge dist/angle polarity
    tev(it+1).parent = I; % initial parent

    [rp, col_check] = check_collision_and_project(tev(I).col,tev(I).vertex,rp,fcs); % for exploration of obstacles
    if isnan(rp) % rp doesnt fit above criteria, reassign random rp & turn back one step
        it = it - 1; 
        continue
    end

    % if col_check == 0
    %    weight = 0.05;
    %    vertex_diff = [rp(1) - parent_vertex(1), rp(2) - parent_vertex(2)];
    %    dist_corr = abs(weight*dist_min);
    %    rp = rp - floor([dist_corr*sin(theta) dist_corr*cos(theta)]);
    %    if any(rp<0)
    %        return
    %    end
    % end

    tev(it+1).vertex = rp; % update
    tev(it+1).col = col_check;
    
    goal_dist = cal_dist(end_pos,tev(it+1).vertex);
    
    plot(tev(it+1).vertex(2),tev(it+1).vertex(1),'o','Color','b','LineWidth',1)
    line([tev(I).vertex(2),tev(it+1).vertex(2)],...
        [tev(I).vertex(1),tev(it+1).vertex(1)],'LineWidth',1,'Color','b')
    drawnow

    if goal_dist <= 50
        current_config = it + 1;
        while current_config ~= 1
            parent_config = tev(current_config).parent;
            line([tev(parent_config).vertex(2),tev(current_config).vertex(2)],...
                [tev(parent_config).vertex(1),tev(current_config).vertex(1)],...
                'Color','r','LineWidth',2)
            plot(tev(parent_config).vertex(2),tev(parent_config).vertex(1),...
                'Marker','diamond','MarkerFaceColor','c','MarkerSize',7.5)
            drawnow
            current_config = parent_config;
        end
        disp('target reached within iteration cap')
        return
    end

end

if it == il
    disp('iteration cap reached, no convergence: no connection present')
end
