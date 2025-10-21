clear
clc
close all

%% IMAGE ACQUISITION AND DATA INTERPRETATION
addpath("testmaps\")
map_rgb = imread('map_2_d.png'); % read the image
map_rgb_resized = imresize(map_rgb,[30 30]); % scale the image
[gr_row, gr_col, r_row, r_col, mask, num] ...
    = color_coordinates(map_rgb_resized); % identify regions of colors
BW = imbinarize(map_rgb_resized(:,:,1),0.85); % binarize the image for processing

imshow(map_rgb_resized) % show the rgb image
plot_map(BW) % plot the BW scatter map

%% VARIABLE INITIALIZATION FOR PATH FINDING ALGORITHM
tic
G = -1*ones(size(BW,1)*size(BW,2)); % dependency matrix from dependency tree
dist = inf*ones(size(BW,1)*size(BW,2),1); % distance to each node from starting node
prec = inf*ones(size(BW,1)*size(BW,2),1); % wtf
nodelist = -1*ones(size(BW,1)*size(BW,2),1); %list of nodes

% dependency matrix is created considering the relation of every node to
% its neighbouring nodes, dependency matrix differs for transeversal and
% diagonal movement allowability in the path finding algorithm

for i=1:size(BW,1)
    for j=1:size(BW,2)
        if BW(i,j)==1 
            %itself
            G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j)=0;

            %down
            if i+1 >0 && i+1<=size(BW,1) && BW(i+1,j)==1
                G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j)=1;
            end

            %up
            if i-1 >0 && i-1<=size(BW,1) && BW(i-1,j)==1
                G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j)=1;
            end

            %right
            if j+1 >0 && j+1<=size(BW,2) && BW(i,j+1)==1
                G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j+1)=1;
            end

            %left
            if j-1 >0 && j-1<=size(BW,2) && BW(i,j-1)==1
                G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j-1)=1;
            end

            %downright
            if i+1 >0 && i+1<=size(BW,1) && j+1<=size(BW,2) && ...
                    BW(i+1,j+1)==1
                G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j+1)=sqrt(2);
            end

            %downleft
            if i+1 >0 && j-1>0 && i+1<=size(BW,1) &&  j-1<=size(BW,2) && ...
                    BW(i+1,j-1)==1              
                G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j-1)=sqrt(2);
            end

            %upright
            if i-1 >0 && i-1<=size(BW,1) && j+1 >0 && j+1<=size(BW,2) && ...
                    BW(i-1,j+1)==1
                G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j+1)=sqrt(2);
            end

            %upleft
            if i-1 >0 && j-1>0 && i-1<=size(BW,1) && j-1<=size(BW,2) && ...
                    BW(i-1,j-1)==1                
                G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j-1)=sqrt(2);
            end

        end
    end
end

%% SYSTEM INITIALIZATION FOR PATH FINDING ALGORITHM - DIJKSTRA
start_pos = [gr_row gr_col]; % (x,y) coordinates of start position taken from image acq
goal_pos = [r_row r_col]; % (x,y) coordinates of goal position taken from image acq

start = (start_pos(1)-1)*size(BW,1)+start_pos(2); % start node
goal = (goal_pos(1)-1)*size(BW,1)+goal_pos(2); % end node

plot_map(BW)

plot(start_pos(2),start_pos(1),'sg','MarkerFaceColor','g') % start on map
plot(goal_pos(2),goal_pos(1),'sr','MarkerFaceColor','r') % goal on map

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% A* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dist(start) = 0; % inital distance defined for the starting node
act_node = start; % active node defined for the starting node
con_node_init = find(G(act_node,:) >= 1); % connection init
nodelist(con_node_init,1) = 1; % nodes to visit init
nodelist(act_node,1) = 0; % nodes that are visited init

while any(nodelist(:,1)==1) && act_node~=goal
    
    [~,con_1]=find(G(act_node,:) == 1); % connected transversal verteces for start
    [~,con_2]=find(G(act_node,:) == sqrt(2)); % connected diagonal verteces for start
    con_nodes = [con_1 con_2]; % connection node init for start
    i_con=length(con_nodes); 

    while i_con>0 % run through all connected nodes of the acting node
        if dist(con_nodes(i_con))>dist(act_node)+1 % if not measured or shorter
            if length(con_nodes) - i_con < length(con_2) % diag sweep
                dist(con_nodes(i_con)) = dist(act_node)+sqrt(2);% calc dist for the new diagonal node
            elseif length(con_nodes) - i_con >= length(con_2) % trans sweep
                dist(con_nodes(i_con)) = dist(act_node)+1;% calc dist for the new transversal node
            end
            prec(con_nodes(i_con))=act_node;
        end
        i_con=i_con-1;
    end
    
    plot_runtime(act_node,BW)

    candidates = find(nodelist == 1);
    dj_cand_dist = dist(candidates);
    cand_pix_row = floor(candidates/size(BW,2)) + 1;
    cand_pix_column = mod(candidates,size(BW,1)) + 1;
    A_weight = 1;
    %hr_cand_row = 0;
    %hr_cand_column = 0;
    %hr_cand_dist = 0;
    hr_cand_dist = sqrt((goal_pos(1,2) - cand_pix_column).^2 + ...
        (goal_pos(1,1) - cand_pix_row).^2);
    hr_init_dist = sqrt((goal_pos(2) - start_pos(2)).^2 + ...
        (goal_pos(1) - start_pos(1)).^2);
    %hr_init_dist = 0;

    %%% evaluate new candidate node & new neighbours
    cost_func = dj_cand_dist + A_weight*hr_cand_dist;
    [min_val,I]=mink(cost_func,length(cost_func));
    %new_nodes=find(dist + hr_init_dist < min_val);
    %new_nodes = find(abs(dist - min_val));
    %new_nodes_distance_sorted = sort(dist(new_nodes));
    
    %[new_nodes,col,v] = find(abs(dist - min_val));
    %[v_sorted, sort_vec] = sort(v);
    %new_nodes_optimality_sorted = new_nodes(sort_vec);
    %new_nodes_distance_sorted = sort(dist(new_nodes));
    %tmp_i=1;
    %while nodelist(new_nodes_optimality_sorted(tmp_i),1)~=1 
    %    tmp_i=tmp_i+1; % first best
    %end    
    %act_node=new_nodes_optimality_sorted(tmp_i); % new node

    new_node_selection_sorted = candidates(I);
    found = 1;
    for i = 1:length(I)   
        if nodelist(new_node_selection_sorted(i)) == 1 && found
            act_node = new_node_selection_sorted(i);
            found = 0;
        end
    end

    nodelist(act_node,1)=0; % visited
    i_con=length(con_nodes);
    while i_con>0
        if nodelist(con_nodes(i_con),1) ~= 0 % if not visited
            nodelist(con_nodes(i_con),1) = 1; % add to visit
        end
        i_con=i_con-1;
    end
end


%%% shortest path
if dist(goal)<inf
    sol_id=goal;
    path=[];
    while(sol_id~=start)
        path=[sol_id path];
        sol_id=prec(sol_id);        
    end
    path=[sol_id path];
    %%% plot shortest path
    for i=1:length(path)
        dr_i=0;
        dr_j=path(i);
        while size(BW,1)<dr_j
            dr_i=dr_i+1;
            dr_j=dr_j-size(BW,1);
        end
        dr_i=(dr_i)+1;
        plot(dr_j,dr_i,'or','MarkerFaceColor','r');
    end
else 
    disp('no solution found')
end
toc
path_dist = dist(goal);
node_num = length(find(nodelist == 0));
node_av = length(find(BW~=0));
node_perc = node_num/node_av*100;
memory_cons = memory().MemUsedMATLAB/(1024^3);
str = sprintf(['Path distance: %f units\nNodes analyzed: %d\nPercentage of ' ...
    'Nodes Utilized: %f%%\nMemory Consumption: %f GB'], path_dist, ...
    node_num, node_perc, memory_cons);
disp(str);

