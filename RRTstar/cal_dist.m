function dist = cal_dist(samp_config, tree_config)
% function for calculation of distance between individual configurations 
% for RRT Algorithm, different distance calculations can be implemented for
% improvements on RRT such as RRT*
   dist = sqrt((tree_config(2) - samp_config(2))^2 + ...
       (tree_config(1) - samp_config(1))^2);
end