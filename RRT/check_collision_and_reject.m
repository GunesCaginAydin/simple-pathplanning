function new_config = check_collision_and_reject(parent, config, config_space)
% function that checks the proposed new configuration on the free
% configuration space and, if the check is not satisfied rejects the
% procedure and proposes another configuration
    while true
        if config_space(config) == 1
            configx_run = parent(1):config(1);
            configy_run = parent(2):config(2);
            configxy = [configx_run' configy_run'];
            if all(config_space(configxy(:,1),configxy(:,2)))
                new_config = config;
                break
            else
                new_config = randi(config_dub);
                continue
            end
        else
            new_config = randi(config_space);
            continue
        end
    end
end