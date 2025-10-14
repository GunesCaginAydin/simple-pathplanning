function maps_rgb_resized = read_and_resize_map_selection(map, resize_arg)
    
    map_rgb = imread(map);         
    maps_rgb_resized = imresize(map_rgb,resize_arg);

end