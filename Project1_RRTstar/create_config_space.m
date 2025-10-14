function [free_config_space, obstacle_coord, obstacle_space, config_space] = create_config_space(rgb_image)
% function for calculation of free configuration space for RRT algorith,
% calculation is through interpretation of singularly reg-green tainted
% otherwise black&white rgb images where start position, goal position and
% obstacles can be uniquely identified, the identification algorithms for
% start and goal positions is through hsv image interpretation whereas the
% for the obstacles is through regionprops function - for more insight
% refer to the custom functions color_coordinates and obstacle_space
%
% this implementation is the basis for other random search algorithms that
% utilize the configuration space, however, care must be exercised for
% different definitions of the physical space for which identification
% schemes may not perform very well

    [obstacle_coord, obj_neg, obj] = create_obstacles(rgb_image);
    free_config_space = obj_neg;
    obstacle_space = obj;
    config_space = ones(size(rgb_image,1),size(rgb_image,2),1);

    function [obstacle_coord, bwff_neg, bwff] = create_obstacles(rgb_image)
        rgb_r = rgb_image(:,:,1);
        rgb_g = rgb_image(:,:,2);
        rgb_b = rgb_image(:,:,3);
        gray_map = rgb2gray(rgb_image);
        bw = gray_map < 50;
        bwf = bwareaopen(bw,50);
        bwff = imfill(bwf,'holes');
        bwff_neg = ~bwff;
        [labeled_image, num_obj] = bwlabel(bwff);
        obstacles = regionprops(labeled_image, 'Perimeter', 'Area', 'PixelList','Centroid');
        obstacle_coord = obstacles.PixelList;
    
        fprintf('%d object(s) found\n\n',num_obj)
        obj_bw = zeros(size(rgb_image,1),size(rgb_image,2),num_obj);
        obj_bw_neg = obj_bw;

        figure(1)
        imshow(rgb_image)

        hold on
        for i = 1:num_obj
            pause(1)
            fprintf('obstacle %d perimeter is %d units\n',i,obstacles(i).Perimeter)
            pause(0.25)
            fprintf('total area of obstacle %d is %d units^2\n\n',i,obstacles(i).Area)
            pause(0.25)
            obj_bw(:,:,i) = labeled_image == i;
            obj_bw(:,:,i) = logical(obj_bw(:,:,i));
            obj_bw_neg(:,:,i) = ~obj_bw(:,:,i);

            rgb_r(obj_bw(:,:,i) == 1) = 255;
            rgb_g(obj_bw(:,:,i) == 1) = 0;
            rgb_b(obj_bw(:,:,i) == 1) = 0;

            rgb_rec = cat(3,rgb_r,rgb_g,rgb_b);
            imshow(rgb_rec)
            
            label = sprintf('Object %d',i);
            t = text(obstacles(i).Centroid(1) - 60, obstacles(i).Centroid(2), label, 'Color', 'b','FontSize',15);
        end
        pause(1)
        delete(t)
    end
    axis on
    return
end