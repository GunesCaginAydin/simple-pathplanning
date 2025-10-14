function [gr_row, gr_col, r_row, r_col, mask, num_col_reg] = color_coordinates(rgb_image)
% function for color identification where an identification scheme is
% proposed through hsv conversion from rgb color coordinates, only works
% for black and white images tainted with monochromatic colors, possible
% improvements should be thought of for more complex cases

    hsv_image = rgb2hsv(rgb_image); % convert read image to hsv
    th = 0.001; % set threshold for color identification 
    mask = hsv_image(:,:,2) > th & hsv_image(:,:,3) > 0.2; % create mask
     
    %[red_channel, green_channel, blue_channel] = imsplit(rgb_image); %%channel id
    
    %propsR = regionprops('table', mask, red_channel, 'MeanIntensity'); % color properties of red
    %propsG = regionprops('table', mask, green_channel, 'MeanIntensity'); % color properties of green
    %propsB = regionprops('table', mask, blue_channel, 'MeanIntensity'); % color properties of blue
    
    mask_log = logical(mask); % mask interpretation by boolean
    connections = bwconncomp(mask_log,8);
    num_col_reg = connections.NumObjects;
    idx_col_reg = regionprops(connections,'SubarrayIdx');
    cg1 = idx_col_reg(1).SubarrayIdx; % color group assignment 1
    cg2 = idx_col_reg(2).SubarrayIdx; % color group assignment 2

    hsv_image(hsv_image<=0) = nan; % discard 0 values on hsv
    [r1,c1] = find(hsv_image(cg1{1},cg1{2},2) == max(hsv_image(cg1{1},cg1{2},2),[],'all'),1);   
    [r2,c2] = find(hsv_image(cg2{1},cg2{2},2) == max(hsv_image(cg2{1},cg2{2},2),[],'all'),1);

    if any(min(hsv_image(:,:,1),[],'all') == hsv_image(cg1{1},cg1{2},1),'all') % cg1 == green

        gr_row = cg1{1}(r1); % green row
        gr_col = cg1{2}(c1); % green column
        r_row = cg2{1}(r2); % red row
        r_col = cg2{2}(c2); % red column

    elseif any(min(hsv_image(:,:,1),[],'all') == hsv_image(cg2{1},cg2{2},1),'all') % cg1 == red

        gr_row = cg2{1}(r2); % green row
        gr_col = cg2{2}(c2); % green column
        r_row = cg1{1}(r1); % red row
        r_col = cg1{2}(c1); % red column
    
    else

        disp('Impossible to Identify the Color Regions, Check Implementation')
        
        return
    
    end
   
end