function [obstacle_coord,obj_bw_neg] = create_obstacles(rgb_map)
    %thresh = graythresh(rgb_map);
    %imshow(rgb_map)
    gray_map = rgb2gray(rgb_map);
    bw = gray_map < 50;
    bwf = bwareaopen(bw,50);
    bwff = imfill(bwf,'holes');
    [labeled_image, num_obj] = bwlabel(bwff);
    obstacles = regionprops(labeled_image, 'Perimeter', 'Area', 'PixelList');
    obstacle_coord = obstacles.PixelList;

    fprintf('%d object(s) found\n',num_obj)
    for i = 1:num_obj
        fprintf('obstacle 1 perimeter is %d\n',obstacles(i).Perimeter)
        pause(0.25)
        fprintf('total area of obstacle 1 is %d\n\n',obstacles(i).Area)
        pause(0.25)
        obj_bw = labeled_image == i;
        obj_bw_neg = ~obg_bw;
        %obj_rgb = 255*uint8(obj_bw);
        %rgb_map(obj_bw,1) = 255;
        %rgb_map(obj_bw,2) = 0;
        %rgb_map(obj_bw,3) = 0;
    end

    % conn = bwconncomp(bwf,8);
    % obstacles = regionprops(conn,'basic');
    % obstacle_coord = {};
    % len = length(obstacles);
    %for i = 1:len
    %    obstacle_coord{i} = obstacles(i).BoundingBox;
    %end
end