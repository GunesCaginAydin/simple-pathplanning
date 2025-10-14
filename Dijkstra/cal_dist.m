function dist = cal_dist(samp_config, tree_config)
   dist = sqrt((tree_config(2) - samp_config(2))^2 + ...
       (tree_config(1) - samp_config(1))^2);
end