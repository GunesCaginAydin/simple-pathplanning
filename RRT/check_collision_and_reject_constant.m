function [new_config, col_check] = check_collision_and_reject_constant(parent_collision ,parent, config, config_space)
% function that checks the proposed new configuration on the free
% configuration space and, if the check is not satisfied, projects the
% configuration to the nearest space on the met obstacle on the edge that
% extends to the parent configuration
rs = size(config_space,1);
cs = size(config_space,2);
weight = 0.6;
projection_min = 100;
    while true
        %config = config - round(50/100*[dist_min*cos(theta) dist_min*sin(theta)]);
        config = round(config);
        if config_space(config(1),config(2)) == 1 % initial check for random point on obstacles
            diff = norm(config - parent);
            theta = atan2(config(1)-parent(1),config(2)-parent(2));
            if diff<=projection_min
                config(1) = 1*config(1);
                config(2) = 1*config(2);
            elseif diff>projection_min
                config(1) = round(parent(1) + weight*diff*sin(theta));
                config(2) = round(parent(2) + weight*diff*cos(theta));
            end
            parent(1) = round(parent(1));
            parent(2) = round(parent(2));
            space_ext_c = abs(parent(2) - config(2));
            space_ext_r = abs(parent(1) - config(1));
            space_ext_param = max([space_ext_c,space_ext_r]);

            configc_run = round(linspace(parent(2),config(2),space_ext_param));
            configr_run = round(linspace(parent(1),config(1),space_ext_param));
            configxy = [configr_run' configc_run'];
            line_idx = sub2ind(size(config_space), configxy(:,1), configxy(:,2));
            
            if all(config_space(line_idx),'all') % collision check for random point
                new_config = [config(1) config(2)]; %NO COLLISION
                col_check = 0;
                return

            % elseif parent_collision == 0
            %     for prc = 1:space_ext_param
            %         if config_space(configxy(prc,1),configxy(prc,2)) == 0
            %             ri = configxy(prc,1); % collision row
            %             ci = configxy(prc,2); % collision column
            %             break
            %         end
            % 
            %     end
            %     rl = ri - 1; rh = ri + 1; cl = ci - 1; ch = ci + 1; % 9-9 search around collision point
            %     if ri == 1
            %         rl = 1;
            %     elseif ri == cs
            %         rh = cs;
            %     end
            % 
            %     if ci == 1
            %         cl = 1;
            %     elseif ci == rs
            %         ch = rs;
            %     end
            % 
            %     rr = rl:rh; cc = cl:ch;
            % 
            %     for pr = 1:length(rr)
            %         for pc = 1:length(cc)
            %             if any(configc_run == cc(pr),'all') && any(configr_run == rr(pc),'all') && config_space(rr(pr),cc(pc)) == 1
            % 
            %                 new_config = [rr(pr),cc(pc)];
            %                 col_check = 1;
            %                 return
            %             end
            %         end
            %     end
            else

                new_config = NaN;
                col_check = NaN;
                return
            end
        else
            new_config = NaN;
            col_check = NaN;
            return
        end
    end
end