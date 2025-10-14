function [maps_rgb, maps_rgb_resized, map_num] = read_and_resize_map(map_folder, resize_arg)
    maps_rgb = cell(10,1); maps_rgb_resized = cell(10);
    folder_cont = dir(map_folder);
    if isempty(folder_cont)
        fig = uifigure;
        uialert(fig,"No maps found","Invalid Folder");
        return
    else 
        map_num = length(folder_cont);
        for map_arg = 3:map_num

            maps_rgb{map_arg} = imread(folder_cont(map_arg).name);         
            maps_rgb_resized{map_arg} = imresize(maps_rgb{map_arg},resize_arg);

        end
    end
    maps_rgb = maps_rgb(~cellfun('isempty',maps_rgb));    
    maps_rgb_resized = maps_rgb_resized(~cellfun('isempty',maps_rgb_resized));

end
