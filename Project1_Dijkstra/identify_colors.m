function [cluster1, cluster2, cluster3] = identify_colors(png_image)

    num_colors = 3;
    L = imsegkmeans(png_image,num_colors);
    B = labeloverlay(png_image,L);

    lab_image = rgb2lab(png_image);
    ab = lab_image(:,:,2:3);
    ab = im2single(ab);
    pixel_labels = imsegkmeans(ab,num_colors,NumAttempts=3);
    L = lab_image(:,:,1);


    mask1 = pixel_labels == 1;
    cluster1 = lab_image.*double(uint8(mask1));

    % L_red = L.*double(mask1);
    % L_red = rescale(L_red);
    % idx_light_red = imbinarize(nonzeros(L_red));
    % red_idx = find(mask1);
    % mask_dark_red = mask1;
    % mask_dark_red(red_idx(idx_light_red)) = 0;
    % red_nuclei = png_image.*uint8(mask_dark_red);

    mask2 = pixel_labels == 2;
    cluster2 = lab_image.*double(uint8(mask2));

    % L_green = L.*double(mask2);
    % L_green = rescale(L_green);
    % idx_light_green = imbinarize(nonzeros(L_green));
    % green_idx = find(mask2);
    % mask_dark_green = mask2;
    % mask_dark_green(green_idx(idx_light_green)) = 0;
    % green_nuclei = png_image.*uint8(mask_dark_green);

    mask3 = pixel_labels == 2;
    cluster3 = lab_image.*double(uint8(mask3));

    % L_blue = L.*double(mask3);
    % L_blue = rescale(L_blue);
    % idx_light_blue = imbinarize(nonzeros(L_blue));
    % blue_idx = find(mask3);
    % mask_dark_blue = mask3;
    % mask_dark_blue(blue_idx(idx_light_blue)) = 0;
    % blue_nuclei = png_image.*uint8(mask_dark_blue);

    
end