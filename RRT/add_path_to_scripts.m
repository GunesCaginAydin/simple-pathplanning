function [script_folder,map_folder] = add_path_to_scripts(~)
% function for finding th path for the problem and scripts
    a1 = questdlg('Please select the folder for RRT scripts',...
        'Script Allocation',...
        'ok', 'abort');
    switch a1
        case 'ok'

        case 'abort'
            fig = uifigure;
            uialert(fig,"Aborted operation","Invalid User Input");
            return
    end
    script_folder = uigetdir(userpath,'Select the path of the script folder');
    addpath(script_folder)
    disp('Script path found and added')

    a2 = questdlg('Please select the folder for test maps',...
        'Script Allocation',...
        'ok', 'abort');
    switch a2
        case 'ok'

        case 'abort'
            fig = uifigure;
            uialert(fig,"Aborted operation","Invalid User Input");
    end
    map_folder = uigetdir(userpath,'Select the path of the map folder');
    addpath(map_folder)
    disp('Map path found and added')

return